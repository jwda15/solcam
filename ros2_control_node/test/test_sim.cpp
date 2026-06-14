// ============================================================================
//  test_sim.cpp  —  ROS 없는 g++ 단독 시뮬 검증 (모드1 선분 유지 + 모드2 회전)
//
//  빌드/실행 (ros2_control_node 디렉토리에서, 한 줄):
//    g++ -std=c++17 -Iinclude src/types.cpp src/pid.cpp src/obstacle_field.cpp
//        src/state_estimator.cpp src/controller_base.cpp
//        src/follow_controller.cpp src/rotate_controller.cpp
//        test/test_sim.cpp -o /tmp/test_sim && /tmp/test_sim
//
//  T1 정적수렴 : 주인 정면 2m, D=2 캡처 → 제자리 유지(정착속도 0)
//  T2 부호     : 주인 우측(az>0) → top_yaw_target 감소(우측 회전)
//  T3 추종     : 주인 이동해도 선분 D·φ 보존
//  T4 끊김     : 트래킹 끊겨도 주인 글로벌 위치 hold
//  T5 모드2    : 주인이 옆으로 이동 → 위치 고정 + 몸체 yaw만 추종
//  T6 오프셋   : heading_offset → 몸체는 비켜보고 OAK-D는 주인 락온 유지
//  T7 손동작 hold: hold_body 중 몸체 정지 + OAK-D 락온 유지, 해제 후 복귀
//  T8 감속 정지  : 주행 중 hold → 가속한계 내에서 부드럽게 0으로 감속
//  T9 PD 감쇠    : 주인 급이동(스텝 입력) 시 D항이 오버슈트 억제 (P 대비)
//  T10 최종 클램프: 과대 명령도 v_max/w_body_max 를 절대 못 넘음 (방향 보존)
//  T11 리드 제한  : 상단yaw 드라이버 정지 시 목표각 폭주 방지 (top_lead_max)
//  T12 선분 클램프: 손동작 이상값에도 D가 seg_d_min~max 를 못 벗어남
//  T13 모드0 teleop: 키보드 목표속도 추종 + 상단yaw 정지 + hold 시 정지
// ============================================================================
#include "control_node/follow_controller.hpp"
#include "control_node/rotate_controller.hpp"
#include "control_node/follow2_controller.hpp"
#include "control_node/orbit_controller.hpp"
#include "control_node/idle_controller.hpp"
#include "control_node/state_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace control_node;

static int g_fail = 0;

static void check(bool ok, const char * name, const char * detail)
{
  std::printf("  %-4s %-30s %s\n", ok ? "PASS" : "FAIL", name, detail);
  if (!ok) { ++g_fail; }
}

// 시뮬 한 스텝: 주인 관측 → 추정 → 제어 → 로봇 적분(완전 추종 가정)
static ControlCommand simStep(IController & ctrl, StateEstimator & est,
                              RobotOdom & robot, double theta_head,
                              const OwnerState & owner,
                              const UserAdjust & adjust, double dt,
                              bool hold_body = false)
{
  bool est_ok = est.update(robot, theta_head, owner);

  ControlInput in;
  in.owner = owner;
  in.robot = robot;
  in.owner_global = est.ownerGlobal();
  in.owner_global_valid = est_ok;
  in.theta_head = theta_head;
  in.adjust = adjust;
  in.hold_body = hold_body;
  in.dt = dt;

  ControlCommand cmd = ctrl.step(in);

  // 몸체 속도(몸체 프레임) → 글로벌 적분
  double c = std::cos(robot.pose.yaw), s = std::sin(robot.pose.yaw);
  robot.pose.x   += (c * cmd.body_vx - s * cmd.body_vy) * dt;
  robot.pose.y   += (s * cmd.body_vx + c * cmd.body_vy) * dt;
  robot.pose.yaw  = wrapAngle(robot.pose.yaw + cmd.body_yaw_rate * dt);
  return cmd;
}

// 주인 글로벌 위치 → 카메라 관측(azimuth/distance) 역산
static OwnerState observe(const RobotOdom & robot, double theta_head,
                          double ox, double oy)
{
  OwnerState o;
  double dx = ox - robot.pose.x, dy = oy - robot.pose.y;
  double bearing_g = std::atan2(dy, dx);
  o.is_detected = true;
  o.distance = std::hypot(dx, dy);
  // bearing_g = yaw + theta_head - azimuth  →  azimuth 역산
  o.azimuth = wrapAngle(robot.pose.yaw + theta_head - bearing_g);
  o.confidence = 1.0;
  return o;
}

// engage 1회 호출 헬퍼
static void engageNow(IController & ctrl, StateEstimator & est,
                      const RobotOdom & robot, double theta_head,
                      const OwnerState & o, const UserAdjust & adj)
{
  ControlInput in;
  in.owner = o; in.robot = robot;
  in.owner_global = est.ownerGlobal(); in.owner_global_valid = true;
  in.theta_head = theta_head; in.adjust = adj;
  ctrl.engage(in);
}


// ---------- T14: 모드3 FOLLOW2(leash) — 거리만 유지, 구도 무시 ----------
static void test_follow2()
{
  std::printf("\n== 모드3 FOLLOW2(leash) ==\n");
  ControllerParams P;
  Follow2Controller f; f.configure(P);
  double dt = 0.02;

  auto run = [&](double dist, double az, double th){
    f.reset();
    OwnerState o; o.is_detected = true; o.distance = dist; o.azimuth = az;
    ControlInput in; in.owner = o; in.theta_head = th; in.dt = dt;
    in.owner_global_valid = false;            // ★odom 없어도 동작해야 함
    f.engage(in);
    ControlCommand c;
    for (int i=0;i<60;i++){ c = f.step(in); }  // 슬루 정상상태
    return c;
  };

  // 밴드(1.0~2.0m) 안 → 정지
  ControlCommand c1 = run(1.6, 0.0, 0.0);
  bool t_band = std::abs(c1.body_vx)<1e-6 && std::abs(c1.body_vy)<1e-6;
  char b[120]; std::snprintf(b,sizeof(b),"dist=1.6(밴드내) vx=%.3f vy=%.3f", c1.body_vx, c1.body_vy);
  check(t_band, "T14a 밴드내 정지", b);

  // 너무 멈(2.5m) → 주인 쪽(정면) 전진. wz=0
  ControlCommand c2 = run(2.5, 0.0, 0.0);
  std::snprintf(b,sizeof(b),"dist=2.5 vx=%.3f(>0=접근) vy=%.3f wz=%.3f", c2.body_vx, c2.body_vy, c2.body_yaw_rate);
  check(c2.body_vx>0.05 && std::abs(c2.body_yaw_rate)<1e-9, "T14b 멀면 접근(wz=0)", b);

  // 너무 가까움(0.7m) → ★후퇴 없이 정지(줄 늘어짐)
  ControlCommand c3 = run(0.7, 0.0, 0.0);
  std::snprintf(b,sizeof(b),"dist=0.7 vx=%.3f vy=%.3f(가까워도 가만)", c3.body_vx, c3.body_vy);
  check(std::abs(c3.body_vx)<1e-6 && std::abs(c3.body_vy)<1e-6, "T14c 가까우면 정지(후퇴X)", b);

  // 주인 우측(az=+0.5) 멀리 → 그 방향으로 이동(vx>0, vy<0). odom 무관.
  ControlCommand c4 = run(2.5, 0.5, 0.0);
  std::snprintf(b,sizeof(b),"az=+0.5 dist=2.5 vx=%.3f vy=%.3f(우측=-)", c4.body_vx, c4.body_vy);
  check(c4.body_vx>0.0 && c4.body_vy<0.0, "T14d 방향(우측 접근)", b);

  // 미탐지 → 정지
  f.reset();
  OwnerState lost; lost.is_detected=false;
  ControlInput in2; in2.owner=lost; in2.dt=dt; f.engage(in2);
  ControlCommand c5; for(int i=0;i<10;i++) c5=f.step(in2);
  std::snprintf(b,sizeof(b),"미탐지 vx=%.3f vy=%.3f", c5.body_vx, c5.body_vy);
  check(std::abs(c5.body_vx)<1e-6 && std::abs(c5.body_vy)<1e-6, "T14e 미탐지 정지", b);
}


// ---------- T15: 모드4 ORBIT(공전) — 반지름 유지하며 천천히 돌기 ----------
static void test_orbit()
{
  std::printf("\n== 모드4 ORBIT(공전) ==\n");
  ControllerParams P; double dt = 0.02;
  OrbitController f; f.configure(P);

  // engage_dist 로 반지름 캡처 후, dist/az 로 정상상태 명령
  auto run = [&](double engage_dist, double dist, double az, double th){
    f.reset();
    OwnerState oe; oe.is_detected=true; oe.distance=engage_dist; oe.azimuth=az;
    ControlInput ein; ein.owner=oe; ein.theta_head=th; ein.dt=dt; ein.owner_global_valid=false;
    f.engage(ein);   // radius = engage_dist
    OwnerState o; o.is_detected=true; o.distance=dist; o.azimuth=az;
    ControlInput in; in.owner=o; in.theta_head=th; in.dt=dt; in.owner_global_valid=false;
    ControlCommand c; for(int i=0;i<80;i++) c=f.step(in); return c;
  };
  char b[140];

  // 반지름에서(정면): 접선 CCW(vy<0) + 진입 상대각 유지 위해 몸체도 공전각속도로
  //   회전. wz_ff = orbit_speed/R = 0.15/1.5 = +0.10 (CCW). vx≈0.
  ControlCommand c1 = run(1.5, 1.5, 0.0, 0.0);
  std::snprintf(b,sizeof(b),"R=dist=1.5 vx=%.3f vy=%.3f(CCW=-) wz=%.3f(=공전각속도0.10)", c1.body_vx, c1.body_vy, c1.body_yaw_rate);
  check(std::abs(c1.body_vx)<0.02 && c1.body_vy<-0.05 && std::abs(c1.body_yaw_rate-0.10)<0.02,
        "T15a 반지름서 접선공전+몸체회전(CCW)", b);

  // 상대각 보정: 진입은 정면(az=0→rel0=0), 이후 주인이 우측(az=+0.3, dir=-0.3)으로
  //   어긋나면 몸체를 CW로 돌려(corr<0이 ff +0.1을 압도) 진입 자세로 복원 → wz<0.
  {
    f.reset();
    OwnerState oe; oe.is_detected=true; oe.distance=1.5; oe.azimuth=0.0;
    ControlInput ein; ein.owner=oe; ein.theta_head=0.0; ein.dt=dt; ein.owner_global_valid=false;
    f.engage(ein);   // rel0 = 0 (정면)
    OwnerState o; o.is_detected=true; o.distance=1.5; o.azimuth=0.3;   // 우측으로 어긋남
    ControlInput in; in.owner=o; in.theta_head=0.0; in.dt=dt; in.owner_global_valid=false;
    ControlCommand c1e; for(int i=0;i<10;i++) c1e=f.step(in);
    std::snprintf(b,sizeof(b),"진입정면 후 az=+0.3 → wz=%.3f(<0=상대각 복원)", c1e.body_yaw_rate);
    check(c1e.body_yaw_rate < -0.05, "T15e 진입 상대각 복원", b);
  }

  // 반지름보다 멈 → 안쪽(+vx) + 접선(-vy)
  ControlCommand c2 = run(1.5, 2.0, 0.0, 0.0);
  std::snprintf(b,sizeof(b),"R=1.5 dist=2.0 vx=%.3f(>0 접근) vy=%.3f", c2.body_vx, c2.body_vy);
  check(c2.body_vx>0.05 && c2.body_vy<-0.02, "T15b 멀면 안쪽+공전", b);

  // 미탐지 → 정지
  f.reset();
  OwnerState lost; lost.is_detected=false;
  ControlInput in2; in2.owner=lost; in2.dt=dt; f.engage(in2);
  ControlCommand c3; for(int i=0;i<10;i++) c3=f.step(in2);
  std::snprintf(b,sizeof(b),"미탐지 vx=%.3f vy=%.3f", c3.body_vx, c3.body_vy);
  check(std::abs(c3.body_vx)<1e-6 && std::abs(c3.body_vy)<1e-6, "T15c 미탐지 정지", b);

  // CW(orbit_ccw=false) → 접선 반대(vy>0)
  ControllerParams Pcw = P; Pcw.orbit_ccw = false;
  OrbitController g; g.configure(Pcw);
  g.reset();
  OwnerState ow; ow.is_detected=true; ow.distance=1.5; ow.azimuth=0.0;
  ControlInput in4; in4.owner=ow; in4.dt=dt; in4.owner_global_valid=false; g.engage(in4);
  ControlCommand c4; for(int i=0;i<80;i++) c4=g.step(in4);
  std::snprintf(b,sizeof(b),"CW vy=%.3f(>0)", c4.body_vy);
  check(c4.body_vy>0.05, "T15d CW 방향 반대", b);
}

int main()
{
  const double dt = 0.02;
  ControllerParams P;   // params.hpp 기본값 사용
  UserAdjust ADJ;       // 조정 없음 (기본)

  std::printf("== 제어기 시뮬 검증 (모드1+모드2+hold) ==\n");

  // ---------- T1: 정적 수렴 (모드1) ----------
  {
    FollowController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;          // 원점, yaw=0
    double ox = 2.0, oy = 0.0;                     // 주인 정면 2m

    OwnerState o = observe(robot, 0.0, ox, oy);
    est.update(robot, 0.0, o);
    engageNow(ctrl, est, robot, 0.0, o, ADJ);

    ControlCommand last;
    for (int i = 0; i < 500; ++i) {
      o = observe(robot, 0.0, ox, oy);
      last = simStep(ctrl, est, robot, 0.0, o, ADJ, dt);
    }
    double v = std::hypot(last.body_vx, last.body_vy);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "D=%.2f, 정착속도=%.4f, pos=(%.3f,%.3f)",
                  ctrl.segDistance(), v, robot.pose.x, robot.pose.y);
    check(std::abs(ctrl.segDistance() - 2.0) < 0.05 && v < 0.01 &&
          std::hypot(robot.pose.x, robot.pose.y) < 0.10,
          "T1 정적수렴", buf);
  }

  // ---------- T2: 상단 yaw 부호 (모드1) ----------
  {
    FollowController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;
    double ox = 2.0, oy = -1.0;                    // 주인 우측 전방

    OwnerState o = observe(robot, 0.0, ox, oy);    // azimuth > 0 (우측)
    est.update(robot, 0.0, o);
    engageNow(ctrl, est, robot, 0.0, o, ADJ);
    simStep(ctrl, est, robot, 0.0, o, ADJ, dt);

    char buf[128];
    std::snprintf(buf, sizeof(buf), "azimuth=%.3f → top_target=%.4f",
                  o.azimuth, ctrl.topYawTarget());
    check(o.azimuth > 0.0 && ctrl.topYawTarget() < 0.0, "T2 부호(우측→CW)", buf);
  }

  // ---------- T3: 추종, 선분 보존 (모드1) ----------
  {
    FollowController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;
    double ox = 2.0, oy = 0.0;

    OwnerState o = observe(robot, 0.0, ox, oy);
    est.update(robot, 0.0, o);
    engageNow(ctrl, est, robot, 0.0, o, ADJ);

    // 주인이 +y로 6초간 0.2 m/s 이동, 이후 4초 정지
    for (int i = 0; i < 500; ++i) {
      if (i < 300) { oy += 0.2 * dt; }
      o = observe(robot, 0.0, ox, oy);
      simStep(ctrl, est, robot, 0.0, o, ADJ, dt);
    }
    double dx = ox - robot.pose.x, dy = oy - robot.pose.y;
    double d_now = std::hypot(dx, dy);
    double phi_now = std::atan2(dy, dx);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "D: 2.00→%.2f, φ: 0.00→%.3f", d_now, phi_now);
    check(std::abs(d_now - 2.0) < 0.10 && std::abs(phi_now) < 0.10,
          "T3 추종(선분 보존)", buf);
  }

  // ---------- T4: 트래킹 끊김 hold ----------
  {
    StateEstimator est;  est.reset();
    RobotOdom robot;  robot.valid = true;
    OwnerState o = observe(robot, 0.0, 2.0, 1.0);
    est.update(robot, 0.0, o);
    Vec2 before = est.ownerGlobal();

    OwnerState lost;  lost.is_detected = false;    // 끊김
    robot.pose.x = 0.5;                            // 로봇은 움직였음
    bool still_valid = est.update(robot, 0.0, lost);
    Vec2 after = est.ownerGlobal();

    char buf[128];
    std::snprintf(buf, sizeof(buf), "hold=(%.2f,%.2f)→(%.2f,%.2f) valid=%d",
                  before.x, before.y, after.x, after.y, still_valid ? 1 : 0);
    check(still_valid && before.x == after.x && before.y == after.y,
          "T4 끊김 hold", buf);
  }

  // ---------- T5: 모드2 — 위치 고정 + 몸체 yaw 추종 ----------
  {
    RotateController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;          // 원점, yaw=0
    double ox = 2.0, oy = 0.0;
    double theta_head = 0.0;                       // 상단 yaw 완전 추종 가정

    OwnerState o = observe(robot, theta_head, ox, oy);
    est.update(robot, theta_head, o);
    engageNow(ctrl, est, robot, theta_head, o, ADJ);

    // 주인이 좌측으로 호를 그리며 이동: 최종 (0, 2) = 방위 +90도
    for (int i = 0; i < 800; ++i) {
      double t = std::min(1.0, i / 400.0);
      double ang = t * M_PI / 2.0;
      ox = 2.0 * std::cos(ang);  oy = 2.0 * std::sin(ang);
      o = observe(robot, theta_head, ox, oy);
      ControlCommand cmd = simStep(ctrl, est, robot, theta_head, o, ADJ, dt);
      theta_head = cmd.top_yaw_target;             // 스텝 드라이버 완전 추종 가정
    }
    double bearing = std::atan2(oy - robot.pose.y, ox - robot.pose.x);
    double yaw_err = wrapAngle(bearing - robot.pose.yaw);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "pos=(%.3f,%.3f), yaw오차=%.3frad",
                  robot.pose.x, robot.pose.y, yaw_err);
    check(std::hypot(robot.pose.x, robot.pose.y) < 1e-6 &&
          std::abs(yaw_err) < 0.06,
          "T5 모드2 회전 추종", buf);
  }

  // ---------- T6: heading_offset — 몸체만 비켜보고 OAK-D는 락온 ----------
  {
    RotateController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;
    double ox = 2.0, oy = 0.0;
    double theta_head = 0.0;

    UserAdjust adj;
    adj.heading_offset = 0.5;                      // 촬영 카메라 +0.5rad 비켜보기

    OwnerState o = observe(robot, theta_head, ox, oy);
    est.update(robot, theta_head, o);
    engageNow(ctrl, est, robot, theta_head, o, adj);

    for (int i = 0; i < 600; ++i) {
      o = observe(robot, theta_head, ox, oy);
      ControlCommand cmd = simStep(ctrl, est, robot, theta_head, o, adj, dt);
      theta_head = cmd.top_yaw_target;
    }
    o = observe(robot, theta_head, ox, oy);
    // 몸체 헤딩 = 주인방위(0) + 0.5, OAK-D azimuth ≈ 0 (락온 유지)
    char buf[128];
    std::snprintf(buf, sizeof(buf), "yaw=%.3f(목표0.5), 잔류 azimuth=%.4f",
                  robot.pose.yaw, o.azimuth);
    check(std::abs(robot.pose.yaw - 0.5) < 0.06 && std::abs(o.azimuth) < 0.05,
          "T6 헤딩 오프셋", buf);
  }

  // ---------- T7: 손동작 hold — 몸체 정지 + 락온 유지 + 해제 후 복귀 ----------
  {
    FollowController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;
    double ox = 2.0, oy = 0.0;
    double theta_head = 0.0;

    OwnerState o = observe(robot, theta_head, ox, oy);
    est.update(robot, theta_head, o);
    engageNow(ctrl, est, robot, theta_head, o, ADJ);

    // 손동작 세션(hold) 중에 주인이 +y로 1m 이동
    double max_speed_during_hold = 0.0;
    for (int i = 0; i < 200; ++i) {
      oy += 1.0 / 200.0;
      o = observe(robot, theta_head, ox, oy);
      ControlCommand cmd = simStep(ctrl, est, robot, theta_head, o, ADJ, dt,
                                   /*hold_body=*/true);
      theta_head = cmd.top_yaw_target;     // 스텝 드라이버 완전 추종 가정
      max_speed_during_hold = std::max(max_speed_during_hold,
        std::hypot(cmd.body_vx, cmd.body_vy) + std::abs(cmd.body_yaw_rate));
    }
    double held_pos = std::hypot(robot.pose.x, robot.pose.y);
    o = observe(robot, theta_head, ox, oy);
    double az_during_hold = std::abs(o.azimuth);   // 락온 유지 확인

    // 세션 해제 → 선분(D=2, φ=0) 목표로 자연 복귀
    for (int i = 0; i < 600; ++i) {
      o = observe(robot, theta_head, ox, oy);
      ControlCommand cmd = simStep(ctrl, est, robot, theta_head, o, ADJ, dt);
      theta_head = cmd.top_yaw_target;
    }
    double dx = ox - robot.pose.x, dy = oy - robot.pose.y;
    double d_after = std::hypot(dx, dy);
    char buf[160];
    std::snprintf(buf, sizeof(buf),
      "hold중 속도=%.4f pos=%.3f az=%.3f → 해제 후 D=%.2f",
      max_speed_during_hold, held_pos, az_during_hold, d_after);
    check(max_speed_during_hold < 1e-9 && held_pos < 1e-9 &&
          az_during_hold < 0.05 && std::abs(d_after - 2.0) < 0.10,
          "T7 손동작 hold·복귀", buf);
  }

  // ---------- T8: 주행 중 hold → 감속 정지 (급정지 아님) ----------
  {
    FollowController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;
    double ox = 5.0, oy = 0.0;                 // 멀리 → 전속(v_max) 주행 유도

    OwnerState o = observe(robot, 0.0, ox, oy);
    est.update(robot, 0.0, o);
    engageNow(ctrl, est, robot, 0.0, o, ADJ);
    ctrl.setSegDistance(2.0, false);           // D=2로 줄여 위치오차 생성

    // 전속 도달까지 주행
    double v_before = 0.0;
    for (int i = 0; i < 50; ++i) {
      o = observe(robot, 0.0, ox, oy);
      ControlCommand cmd = simStep(ctrl, est, robot, 0.0, o, ADJ, dt);
      v_before = std::hypot(cmd.body_vx, cmd.body_vy);
    }

    // hold 시작 → 속도가 단조감소 + 스텝당 감속이 accel 한계 이내
    double v_prev = v_before;
    double max_drop = 0.0;
    int steps_to_stop = -1;
    for (int i = 0; i < 100; ++i) {
      o = observe(robot, 0.0, ox, oy);
      ControlCommand cmd = simStep(ctrl, est, robot, 0.0, o, ADJ, dt, true);
      double v = std::hypot(cmd.body_vx, cmd.body_vy);
      max_drop = std::max(max_drop, v_prev - v);
      if (v < 1e-6 && steps_to_stop < 0) { steps_to_stop = i + 1; }
      v_prev = v;
    }
    double drop_limit = P.body_accel_max * dt * 1.5;   // 한계(+여유 1.5배)
    char buf[160];
    std::snprintf(buf, sizeof(buf),
      "v=%.2f→0 (%d스텝=%.2fs), 스텝당 최대감속=%.4f(한계 %.4f)",
      v_before, steps_to_stop, steps_to_stop * dt, max_drop,
      P.body_accel_max * dt);
    check(v_before > 0.3 && steps_to_stop > 5 && max_drop <= drop_limit &&
          v_prev < 1e-6,
          "T8 감속 정지(점진)", buf);
  }

  // ---------- T9: PD 감쇠 — 스텝 입력 오버슈트 비교 (P vs PD) ----------
  {
    // 시나리오: 정착 상태에서 주인이 +x로 0.5m 급이동(스텝 입력)
    //  → 로봇이 새 목표점(0.5,0)으로 이동. 목표를 지나친 거리(오버슈트) 측정.
    auto run_overshoot = [&](double kd) {
      ControllerParams p = P;  p.kd_pos = kd;
      FollowController ctrl;  ctrl.configure(p);  ctrl.reset();
      StateEstimator est;     est.reset();
      RobotOdom robot;  robot.valid = true;
      double ox = 2.0, oy = 0.0;

      OwnerState o = observe(robot, 0.0, ox, oy);
      est.update(robot, 0.0, o);
      engageNow(ctrl, est, robot, 0.0, o, ADJ);     // D=2 캡처, 목표=(0,0)

      ox = 2.5;                                     // 주인 급이동 → 목표=(0.5,0)
      double overshoot = 0.0;
      for (int i = 0; i < 500; ++i) {
        o = observe(robot, 0.0, ox, oy);
        simStep(ctrl, est, robot, 0.0, o, ADJ, dt);
        overshoot = std::max(overshoot, robot.pose.x - 0.5);
      }
      return overshoot;
    };
    double ov_p  = run_overshoot(0.0);             // P만
    double ov_pd = run_overshoot(P.kd_pos);        // PD (기본 kd)
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "오버슈트 P=%.4fm → PD=%.4fm (kd_pos=%.2f)", ov_p, ov_pd, P.kd_pos);
    check(ov_pd <= ov_p + 1e-9 && ov_pd < 0.05, "T9 PD 감쇠(오버슈트)", buf);
  }

  // ---------- T10: 최종 안전 클램프 (발행 직전 관문) ----------
  {
    ControlCommand c;
    c.body_vx = 1.0;  c.body_vy = 1.0;  c.body_yaw_rate = -5.0;   // 과대 명령
    applySafetyLimits(c, P.v_max, P.w_body_max);
    double v = std::hypot(c.body_vx, c.body_vy);
    bool dir_kept = std::abs(c.body_vx - c.body_vy) < 1e-12;      // 방향 보존
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "v 1.41→%.3f (한계 %.2f), wz -5→%.2f, 방향보존=%d",
      v, P.v_max, c.body_yaw_rate, dir_kept ? 1 : 0);
    check(std::abs(v - P.v_max) < 1e-9 && c.body_yaw_rate == -P.w_body_max &&
          dir_kept,
          "T10 최종 속도 클램프", buf);
  }

  // ---------- T11: 상단yaw 리드 제한 — 드라이버 정지 시 목표 폭주 방지 ----------
  {
    RotateController ctrl;  ctrl.configure(P);  ctrl.reset();
    StateEstimator est;     est.reset();
    RobotOdom robot;  robot.valid = true;
    double ox = 0.0, oy = -2.0;                // 주인 우측 90도 (azimuth 큼)
    double theta_head = 0.0;                   // ★스테이지 고장 가정: 안 움직임

    OwnerState o = observe(robot, theta_head, ox, oy);
    est.update(robot, theta_head, o);
    engageNow(ctrl, est, robot, theta_head, o, ADJ);

    for (int i = 0; i < 500; ++i) {            // 10초간 azimuth 계속 큼
      o = observe(robot, theta_head, ox, oy);
      ControlInput in;
      in.owner = o; in.robot = robot;
      in.owner_global = est.ownerGlobal(); in.owner_global_valid = true;
      in.theta_head = theta_head; in.adjust = ADJ; in.dt = dt;
      ctrl.step(in);                           // theta_head 갱신 안 함(고장)
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf),
      "10초 후 top_target=%.3f (리드한계 ±%.1f, 무제한이면 -8.0)",
      ctrl.topYawTarget(), P.top_lead_max);
    check(std::abs(ctrl.topYawTarget()) <= P.top_lead_max + 1e-9,
          "T11 상단yaw 리드 제한", buf);
  }

  // ---------- T12: 손동작 선분 거리 클램프 ----------
  {
    FollowController ctrl;  ctrl.configure(P);  ctrl.reset();
    ctrl.setSegDistance(10.0, false);            // 이상값(절대 10m)
    double d_hi = ctrl.segDistance();
    ctrl.setSegDistance(-20.0, true);            // 이상값(증분 -20m)
    double d_lo = ctrl.segDistance();
    char buf[128];
    std::snprintf(buf, sizeof(buf), "10m→%.1f(상한 %.1f), -20m→%.1f(하한 %.1f)",
                  d_hi, P.seg_d_max, d_lo, P.seg_d_min);
    check(d_hi == P.seg_d_max && d_lo == P.seg_d_min, "T12 선분 거리 클램프", buf);
  }

  // ---------- T13: 모드0 IDLE — teleop 추종 / 상단yaw 정지 / hold 정지 ----------
  {
    IdleController ctrl;  ctrl.configure(P);  ctrl.reset();
    // requiresOwner=false (추적 없이 동작)
    bool need_owner = ctrl.requiresOwner();

    ControlInput in;
    in.robot.valid = true;
    in.theta_head = 0.7;            // 상단 스테이지 임의 각
    in.dt = dt;
    ctrl.engage(in);

    // teleop 전진+좌 (대각선). 가속제한 때문에 여러 스텝 후 목표 도달.
    in.teleop_vx = 0.3; in.teleop_vy = 0.2; in.teleop_wz = 0.0;
    ControlCommand cmd;
    for (int i = 0; i < 200; ++i) { cmd = ctrl.step(in); }
    bool reaches = std::abs(cmd.body_vx - 0.3) < 1e-3 &&
                   std::abs(cmd.body_vy - 0.2) < 1e-3;
    bool top_stopped = (cmd.top_yaw_active == false);

    // hold_body=true → 몸체 감속 정지
    in.hold_body = true;
    for (int i = 0; i < 200; ++i) { cmd = ctrl.step(in); }
    bool held = std::hypot(cmd.body_vx, cmd.body_vy) < 1e-9;

    char buf[160];
    std::snprintf(buf, sizeof(buf),
      "need_owner=%d teleop(vx=%.2f vy=%.2f) top_active=%d hold후속도=%.3f",
      need_owner ? 1 : 0, cmd.body_vx, cmd.body_vy,
      cmd.top_yaw_active ? 1 : 0, std::hypot(cmd.body_vx, cmd.body_vy));
    check(!need_owner && reaches && top_stopped && held,
          "T13 모드0 teleop", buf);
  }

  test_follow2();
  test_orbit();
  std::printf("== 결과: %s ==\n", g_fail == 0 ? "ALL PASS" : "FAIL 있음");
  return g_fail;
}

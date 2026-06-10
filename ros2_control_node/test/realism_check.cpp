// ============================================================================
//  realism_check.cpp  —  실제 출력 제어값(vx/vy/wz/top_yaw) 점검 (ROS 없음)
//
//  게인/한계를 바꿔가며 "현실적인 값이 나오는지" 숫자로 확인하는 튜닝 도구.
//  빌드/실행 (ros2_control_node 에서 한 줄):
//    g++ -std=c++17 -Iinclude src/types.cpp src/pid.cpp src/obstacle_field.cpp \
//        src/state_estimator.cpp src/controller_base.cpp \
//        src/follow_controller.cpp src/rotate_controller.cpp \
//        test/realism_check.cpp -o /tmp/realism && /tmp/realism
// ============================================================================
#include "control_node/follow_controller.hpp"
#include "control_node/rotate_controller.hpp"
#include "control_node/state_estimator.hpp"
#include <cmath>
#include <cstdio>
#include <initializer_list>
using namespace control_node;

static OwnerState observe(const RobotOdom& r, double th, double ox, double oy){
  OwnerState o; double dx=ox-r.pose.x, dy=oy-r.pose.y;
  double bg=std::atan2(dy,dx);
  o.is_detected=true; o.distance=std::hypot(dx,dy);
  o.azimuth=wrapAngle(r.pose.yaw+th-bg); o.confidence=1.0; return o;
}
// 로봇 고정한 채 N스텝 → 슬루 다 찬 정상상태 명령
static ControlCommand steady(IController& c, StateEstimator& e, RobotOdom r,
                             double th, double ox, double oy, double dt){
  ControlCommand cmd; UserAdjust adj;
  for(int i=0;i<60;i++){
    OwnerState o=observe(r,th,ox,oy);
    bool ok=e.update(r,th,o);
    ControlInput in; in.owner=o; in.robot=r; in.owner_global=e.ownerGlobal();
    in.owner_global_valid=ok; in.theta_head=th; in.adjust=adj; in.dt=dt;
    cmd=c.step(in);
  }
  return cmd;
}

int main(){
  ControllerParams p;  // params.hpp 기본값
  double dt=0.02;
  std::printf("[기본 파라미터] v_max=%.2f w_body_max=%.2f kp_pos=%.2f kd_pos=%.2f kp_byaw=%.2f accel=%.2f\n\n",
              p.v_max,p.w_body_max,p.kp_pos,p.kd_pos,p.kp_byaw,p.body_accel_max);

  // ---- FOLLOW: 주인 정면 1.5m, D=1.5 캡처(로봇 원점 +x 향함) ----
  RobotOdom r; r.valid=true; r.pose={0,0,0};
  double th=0.0;
  StateEstimator est; FollowController f; f.configure(p); 
  { OwnerState o=observe(r,th,1.5,0.0); bool ok=est.update(r,th,o);
    ControlInput in; in.owner=o; in.robot=r; in.owner_global=est.ownerGlobal();
    in.owner_global_valid=ok; in.theta_head=th; in.dt=dt; f.reset(); f.engage(in); }

  struct Case{const char*name; double ox,oy;};
  Case cs[]={
    {"제자리(주인 1.5m 정면)      ",1.5,0.0},
    {"주인 0.5m 전진(2.0m)        ",2.0,0.0},
    {"주인 0.5m 후진(1.0m)        ",1.0,0.0},
    {"주인 0.5m 좌(1.5,+0.5)      ",1.5,0.5},
    {"주인 0.5m 우(1.5,-0.5)      ",1.5,-0.5},
    {"주인 2m 전진(3.5m, 큰오차)  ",3.5,0.0},
    {"주인 3cm 미세(<데드존)      ",1.53,0.0},
  };
  std::printf("FOLLOW(모드1)  ┌ vx(+전) vy(+좌) m/s   wz(+좌) rad/s   top_yaw rad\n");
  for(auto&c:cs){
    StateEstimator e2; FollowController f2; f2.configure(p);
    { OwnerState o=observe(r,th,1.5,0.0); bool ok=e2.update(r,th,o);
      ControlInput in; in.owner=o; in.robot=r; in.owner_global=e2.ownerGlobal();
      in.owner_global_valid=ok; in.theta_head=th; in.dt=dt; f2.reset(); f2.engage(in); }
    ControlCommand cmd=steady(f2,e2,r,th,c.ox,c.oy,dt);
    std::printf("  %s vx=%+.3f vy=%+.3f   wz=%+.3f      top=%+.3f\n",
                c.name,cmd.body_vx,cmd.body_vy,cmd.body_yaw_rate,cmd.top_yaw_target);
  }

  // ---- ROTATE(모드2): 위치고정, 주인 옆 ----
  std::printf("\nROTATE(모드2)  주인을 옆에 두면 vx=vy=0, wz만:\n");
  RotateController rc; rc.configure(p);
  for(double ang: {0.3, -0.3, 0.8}){
    double ox=1.5*std::cos(ang), oy=1.5*std::sin(ang);
    StateEstimator e3; rc.reset();
    { OwnerState o=observe(r,th,ox,oy); bool ok=e3.update(r,th,o);
      ControlInput in; in.owner=o; in.robot=r; in.owner_global=e3.ownerGlobal();
      in.owner_global_valid=ok; in.theta_head=th; in.dt=dt; rc.engage(in); }
    ControlCommand cmd=steady(rc,e3,r,th,ox,oy,dt);
    std::printf("  주인 방위 %+.2f rad(%.0f도): vx=%+.3f vy=%+.3f wz=%+.3f\n",
                ang,ang*57.3,cmd.body_vx,cmd.body_vy,cmd.body_yaw_rate);
  }
  return 0;
}

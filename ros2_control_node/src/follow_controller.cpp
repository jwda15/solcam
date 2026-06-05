// ============================================================================
//  follow_controller.cpp  —  모드1(FOLLOW) "선분 유지" 제어기 정의
//  (개념/설계 설명: include/control_node/follow_controller.hpp)
// ============================================================================
#include "control_node/follow_controller.hpp"

#include <algorithm>
#include <cmath>

namespace control_node
{

void FollowController::onReset()
{
  engaged_ = false;
  pid_x_.reset();
  pid_y_.reset();
}

// 위치 PD 세팅: 축별(x/y) 동일 게인. 출력은 ±v_max (벡터 크기는 별도 제한).
//  데드존은 0 — 도착 판정은 벡터 크기(err_norm <= pos_dead)로 일괄 처리.
void FollowController::onConfigure()
{
  pid_x_.setGains(params_.kp_pos, 0.0, params_.kd_pos);
  pid_x_.setLimits(-params_.v_max, params_.v_max);
  pid_y_.setGains(params_.kp_pos, 0.0, params_.kd_pos);
  pid_y_.setLimits(-params_.v_max, params_.v_max);
}

// 모드 진입 1회: 현재 주인-로봇 선분(거리 D, 글로벌 각도 φ)을 목표로 고정.
void FollowController::engage(const ControlInput & in)
{
  engageCommon(in);   // 상단 yaw 목표각 = 현재 스테이지 각

  if (in.owner_global_valid && in.robot.valid) {
    double dx = in.owner_global.x - in.robot.pose.x;
    double dy = in.owner_global.y - in.robot.pose.y;
    seg_distance_ = std::hypot(dx, dy);
    seg_angle_    = std::atan2(dy, dx);      // 로봇→주인 글로벌 방향 φ
    // 거리가 비정상(0 근처)이면 파라미터 기본값으로
    if (seg_distance_ < 1e-3) { seg_distance_ = params_.seg_distance; }
    engaged_ = true;
  } else {
    // 진입 시 추정이 없으면 파라미터 거리 + 현재 헤딩 방향으로 임시 세팅
    seg_distance_ = params_.seg_distance;
    seg_angle_    = in.robot.valid ? in.robot.pose.yaw : 0.0;
    engaged_ = false;   // 첫 유효 프레임에서 다시 캡처
  }
}

// 한 스텝 제어. 흐름:
//   상단 yaw 갱신(주인 락온) + 리프트 → 주행 가능 판정 → 몸체 vx/vy(선분 추종)
//   → 몸체 yaw(주인 방향 + 오프셋) → 슬루레이트 제한 → 명령 반환
ControlCommand FollowController::step(const ControlInput & in)
{
  ControlCommand cmd;

  // 진입 때 추정이 없었다면, 첫 유효 프레임에서 선분을 캡처
  if (!engaged_ && in.owner_global_valid && in.robot.valid) {
    engage(in);
  }

  // ----- 1) 상단 yaw(OAK-D 주인 락온) + 리프트 (공통 부품) -----
  trackTopYaw(in, cmd);
  applyLift(in, cmd);

  // 정지 처리 (상단yaw 락온은 위에서 이미 갱신 — 정지 중에도 계속 본다):
  //  1) 추정/오도메트리 미확보 → 즉시 정지 (안전 우선)
  //  2) 손동작 세션(hold_body) → 가속한계로 감속 정지 (부드럽게)
  //  선분 목표는 불변이므로 hold 가 풀리면 위치오차로 자연 복귀.
  bool sensors_ok = in.owner_global_valid && in.robot.valid && engaged_;
  if (!sensors_ok) {
    stopBodyAndClearSlew(cmd);
    pid_x_.reset();  pid_y_.reset();   // 오차 이력 무효 → 재개 시 D항 튐 방지
    return cmd;
  }
  if (in.hold_body) {
    stopBodySmooth(cmd, in.dt);
    pid_x_.reset();  pid_y_.reset();
    return cmd;
  }

  // ----- 2) 몸체 vx/vy: 선분 끝점(target) 추종 -----
  double vx_b = 0.0, vy_b = 0.0;
  computeBodyVelocity(in, vx_b, vy_b);

  // ----- 3) 몸체 yaw: (주인 방향 또는 선분각) + 헤딩 오프셋 -----
  //  offset=0 이면 촬영 카메라(몸체 정면)도 주인을 정면으로.
  //  offset≠0 이면 OAK-D는 주인 락온 유지, 촬영 카메라만 다른 방향.
  //  주인이 코앞(방위각 특이점)이면 yaw는 현 헤딩 유지(wz=0).
  double wz = 0.0;
  if (!(params_.face_owner && ownerTooClose(in))) {
    double base_heading = params_.face_owner ? ownerBearing(in) : seg_angle_;
    wz = yawRateToHeading(wrapAngle(base_heading + in.adjust.heading_offset), in);
  }

  // ----- 4) 슬루레이트(가속) 제한 + 출력 (공통 부품) -----
  writeBodyCommand(cmd, vx_b, vy_b, wz, in.dt);
  return cmd;
}

// ============================================================================
//  손동작 조정 / 접근자
// ============================================================================
void FollowController::setSegDistance(double value, bool delta)
{
  double d = delta ? seg_distance_ + value : value;
  // 손동작 경로는 범위 클램프: supervisor 가 이상값을 보내도
  // 로봇이 과도하게 붙거나 멀어지지 않게. (engage 캡처는 클램프 안 함)
  seg_distance_ = std::clamp(d, params_.seg_d_min, params_.seg_d_max);
}

void FollowController::setSegAngle(double value, bool delta)
{
  seg_angle_ = wrapAngle(delta ? seg_angle_ + value : value);
}

double FollowController::segDistance() const { return seg_distance_; }
double FollowController::segAngle() const    { return seg_angle_; }

// ============================================================================
//  내부 계산
// ============================================================================

// 몸체 vx/vy: 목표 로봇 위치 = 주인 글로벌 - D*(cosφ, sinφ).
//  글로벌 위치오차에 축별 PD(kp_pos/kd_pos)를 걸고, 벡터 크기를 v_max로
//  제한한 뒤 몸체 프레임으로 회전변환. (가속 제한은 호출부 슬루가 담당)
//  D항: 오차가 줄어드는 속도에 비례해 미리 감속 → 목표점 오버슈트 억제.
void FollowController::computeBodyVelocity(const ControlInput & in,
                                           double & vx_b, double & vy_b)
{
  vx_b = 0.0;
  vy_b = 0.0;

  // 목표점(선분 끝)
  double tx = in.owner_global.x - seg_distance_ * std::cos(seg_angle_);
  double ty = in.owner_global.y - seg_distance_ * std::sin(seg_angle_);

  // 글로벌 위치오차
  double ex_g = tx - in.robot.pose.x;
  double ey_g = ty - in.robot.pose.y;
  double err_norm = std::hypot(ex_g, ey_g);
  if (err_norm <= params_.pos_dead) {
    // 도착 — 정지. PD 이력도 비워 재출발 시 D항 튐 방지.
    pid_x_.reset();
    pid_y_.reset();
    return;
  }

  // 글로벌 속도 희망 (축별 PD)
  double vx_g = pid_x_.update(ex_g, in.dt);
  double vy_g = pid_y_.update(ey_g, in.dt);

  // 벡터 크기 제한 (축별 클램프만으론 대각선이 v_max*√2 가 될 수 있음)
  double v_norm = std::hypot(vx_g, vy_g);
  if (v_norm > params_.v_max) {
    vx_g *= params_.v_max / v_norm;
    vy_g *= params_.v_max / v_norm;
  }

  // 글로벌 → 몸체 프레임 회전변환 (R(-yaw))
  double c = std::cos(in.robot.pose.yaw);
  double s = std::sin(in.robot.pose.yaw);
  vx_b =  c * vx_g + s * vy_g;
  vy_b = -s * vx_g + c * vy_g;
}

}  // namespace control_node

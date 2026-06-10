// ============================================================================
//  follow2_controller.cpp  —  Follow2Controller(모드3, leash) 정의
//  (개념/특징: include/control_node/follow2_controller.hpp)
// ============================================================================
#include "control_node/follow2_controller.hpp"

#include <algorithm>
#include <cmath>

namespace control_node
{

void Follow2Controller::onConfigure()
{
  leash_distance_ = params_.leash_distance;
}

void Follow2Controller::engage(const ControlInput & in)
{
  engageCommon(in);   // 상단 yaw 목표각을 현재 스테이지 각으로 초기화
}

void Follow2Controller::setLeashDistance(double value, bool delta)
{
  double d = delta ? leash_distance_ + value : value;
  leash_distance_ = std::clamp(d, params_.seg_d_min, params_.seg_d_max);
}

ControlCommand Follow2Controller::step(const ControlInput & in)
{
  ControlCommand cmd;

  // 상단 yaw(주인 락온) + 리프트 (공통 부품). odom 불필요.
  trackTopYaw(in, cmd);
  applyLift(in, cmd);

  // 주인 미탐지 → 즉시 정지 / 손동작 세션 → 감속 정지
  if (!in.owner.is_detected) { stopBodyAndClearSlew(cmd); return cmd; }
  if (in.hold_body)          { stopBodySmooth(cmd, in.dt); return cmd; }

  // ----- 거리 오차 + 널널한 데드존 -----
  //  밴드(±leash_dead) 안이면 정지. 밖이면 밴드 끝을 기준으로 P제어.
  double err = in.owner.distance - leash_distance_;   // +면 너무 멈(끌려감)
  double speed = 0.0;
  if (std::abs(err) > params_.leash_dead) {
    double e = err - std::copysign(params_.leash_dead, err);   // 밴드 끝에서 0
    speed = std::clamp(params_.kp_leash * e, -params_.v_max, params_.v_max);
  }

  // ----- 몸체 기준 주인 방향 -----
  //  글로벌 bearing = robot.yaw + theta_head - azimuth
  //  몸체프레임 방향 = bearing - robot.yaw = theta_head - azimuth
  double dir = wrapAngle(in.theta_head - in.owner.azimuth);
  double vx_b = speed * std::cos(dir);   // +speed = 주인 쪽으로
  double vy_b = speed * std::sin(dir);

  // 구도(헤딩) 제어 없음: wz=0. 가속(슬루) 제한만 걸어 출력.
  writeBodyCommand(cmd, vx_b, vy_b, 0.0, in.dt);
  return cmd;
}

}  // namespace control_node

// ============================================================================
//  orbit_controller.cpp  —  OrbitController(모드4, 공전) 정의
//  (개념/특징: include/control_node/orbit_controller.hpp)
// ============================================================================
#include "control_node/orbit_controller.hpp"

#include <algorithm>
#include <cmath>

namespace control_node
{

void OrbitController::onConfigure()
{
  orbit_radius_ = params_.seg_distance;   // 폴백(진입 때 미검출 대비)
}

void OrbitController::engage(const ControlInput & in)
{
  engageCommon(in);            // 상단 yaw 목표각 초기화
  captured_ = false;
  // 진입(선택) 시점에 주인이 보이면 그 거리를 반지름으로 캡처
  if (in.owner.is_detected && in.owner.distance > 1e-3) {
    orbit_radius_ = in.owner.distance;
    captured_ = true;
  }
}

void OrbitController::setOrbitRadius(double value, bool delta)
{
  double d = delta ? orbit_radius_ + value : value;
  orbit_radius_ = std::clamp(d, params_.seg_d_min, params_.seg_d_max);
}

ControlCommand OrbitController::step(const ControlInput & in)
{
  ControlCommand cmd;

  // 상단 yaw(주인 락온) + 리프트 (공통). odom 불필요.
  trackTopYaw(in, cmd);
  applyLift(in, cmd);

  if (!in.owner.is_detected) { stopBodyAndClearSlew(cmd); return cmd; }

  // 진입 때 미검출이었으면 첫 유효 프레임에서 반지름 캡처
  if (!captured_ && in.owner.distance > 1e-3) {
    orbit_radius_ = in.owner.distance;
    captured_ = true;
  }

  if (in.hold_body) { stopBodySmooth(cmd, in.dt); return cmd; }

  // 몸체기준 주인방향 (글로벌 bearing - robot.yaw = theta_head - azimuth)
  double dir = wrapAngle(in.theta_head - in.owner.azimuth);

  // 반지름 유지(양방향 P): +오차(너무 멈) → 주인 쪽(+dir 방향)
  double v_rad = params_.kp_orbit_r * (in.owner.distance - orbit_radius_);

  // 접선 공전(천천히): CCW(+) 접선 단위벡터 = (sin dir, -cos dir)
  double sgn   = params_.orbit_ccw ? 1.0 : -1.0;
  double v_tan = sgn * params_.orbit_speed;

  // 합성 (반지름은 dir 방향, 접선은 그 수직):
  double vx_b = v_rad * std::cos(dir) + v_tan * std::sin(dir);
  double vy_b = v_rad * std::sin(dir) - v_tan * std::cos(dir);

  // 몸체 yaw 제어 없음(wz=0): 게걸음 공전, 프레이밍은 상단 yaw 카메라가.
  //  (총 속도 v_max 상한은 발행부 applySafetyLimits 가 벡터크기로 강제)
  writeBodyCommand(cmd, vx_b, vy_b, 0.0, in.dt);
  return cmd;
}

}  // namespace control_node

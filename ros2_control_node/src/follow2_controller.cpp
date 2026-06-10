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

  // ----- 거리: 진짜 "줄(leash)"처럼 단방향 -----
  //  정해진 거리(leash_distance)+데드존(leash_dead)을 "넘을 때만" 끌려간다.
  //  가까우면(밴드 안이거나 더 가까워도) 줄이 늘어져 가만히 있는다(후퇴 없음).
  double over = in.owner.distance - (leash_distance_ + params_.leash_dead);
  double speed = 0.0;
  if (over > 0.0) {
    speed = std::min(params_.kp_leash * over, params_.v_max);  // 항상 +(접근)
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

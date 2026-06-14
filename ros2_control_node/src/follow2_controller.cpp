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
  rel_captured_ = false;
  if (in.owner.is_detected) {   // 진입 시 주인 기준 상대각 캡처(유지 기준)
    rel0_ = wrapAngle(in.theta_head - in.owner.azimuth);
    rel_captured_ = true;
  }
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

  // ----- 몸체 yaw: 진입 때의 '주인 기준 상대각(rel0)' 유지 -----
  //  주인이 옆으로 이동해 dir 이 바뀌면 몸체를 같이 돌려, 진입 때 주인을
  //  향하던(또는 특정 면을 보이던) 자세를 계속 유지. base PD(yaw_pid_) 재사용.
  if (!rel_captured_) { rel0_ = dir; rel_captured_ = true; }
  double wz = yaw_pid_.update(wrapAngle(dir - rel0_), in.dt);

  writeBodyCommand(cmd, vx_b, vy_b, wz, in.dt);
  return cmd;
}

}  // namespace control_node

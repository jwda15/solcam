// ============================================================================
//  state_estimator.cpp  —  StateEstimator 정의
//  (선언/합성식 설명: include/control_node/state_estimator.hpp)
// ============================================================================
#include "control_node/state_estimator.hpp"

#include <cmath>

namespace control_node
{

void StateEstimator::reset()
{
  have_owner_ = false;
  owner_g_ = Vec2{};
}

bool StateEstimator::update(const RobotOdom & robot, double theta_head,
                            const OwnerState & owner)
{
  // 오도메트리가 없으면 글로벌 합성 불가 (제어는 호출부에서 폴백 처리)
  if (!robot.valid) { return have_owner_; }

  if (owner.is_detected) {
    // 주인을 봤을 때만 글로벌 위치를 새로 계산.
    //   bearing_g = 로봇yaw + 상단yaw각 - azimuth (부호 규약은 헤더 참조)
    double bearing_g = wrapAngle(robot.pose.yaw + theta_head - owner.azimuth);
    owner_g_.x = robot.pose.x + owner.distance * std::cos(bearing_g);
    owner_g_.y = robot.pose.y + owner.distance * std::sin(bearing_g);
    have_owner_ = true;
  }
  // 미탐지면 직전 owner_g_ 를 그대로 유지(hold). 바닥 고정이라 유효.
  return have_owner_;
}

bool StateEstimator::hasOwner() const
{
  return have_owner_;
}

Vec2 StateEstimator::ownerGlobal() const
{
  return owner_g_;
}

}  // namespace control_node

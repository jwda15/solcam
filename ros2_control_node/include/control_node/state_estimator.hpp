// ============================================================================
//  state_estimator.hpp  —  주인의 글로벌 위치 추정 (선언)
//
//  책임: 매 프레임 "주인이 글로벌 평면에서 어디 있는가"(Vec2)를 추정.
//        제어기(FollowController)는 이 글로벌 위치만 받아 선분 유지를 계산한다.
//
//  입력 3종을 합성:
//    1) 로봇 글로벌 포즈 (휠 오도메트리)        : Pose2D
//    2) 상단 yaw 스테이지 각 theta_head         : rad (스텝모터라 정확)
//    3) 카메라가 본 주인 (azimuth, distance)    : OwnerState
//
//  합성식 (주인 글로벌 방위):
//    카메라 광축의 글로벌 방향 = robot.yaw + theta_head
//    주인은 광축 기준 azimuth(우측+) 만큼 → 글로벌 기준 (광축 - azimuth)
//      (azimuth 부호: 우측+ = 시계 = CCW 음수 이므로 부호 반전)
//    bearing_g = robot.yaw + theta_head - azimuth
//    owner_g.x = robot.x + distance * cos(bearing_g)
//    owner_g.y = robot.y + distance * sin(bearing_g)
//
//  ── 휠 오도메트리 ↔ 트래킹 상호 보완 (지금은 1단계만, 자리 마련) ──
//   * 트래킹 끊김(미탐지/저신뢰): 직전 주인 글로벌 위치를 "그대로 유지"한다.
//     주인 글로벌 위치는 바닥 고정이라, 로봇이 움직여도 글로벌 좌표는 안 변함
//     → 오도메트리가 로봇 이동을 알고 있으니, 다음 프레임 제어가 자연 복귀.
//     (추측항법: 별도 적분 없이 "마지막 글로벌 위치 hold"로 충분)
//   * [TODO/2단계] 주인이 정지로 판단될 때, 관측된 주인 글로벌 위치의 변화를
//     역이용해 오도메트리 yaw 드리프트를 보정(주인=느슨한 랜드마크).
//     이 클래스만 교체하면 되도록 인터페이스를 여기로 격리해 둠.
//
//  (정의: src/state_estimator.cpp)
// ============================================================================
#ifndef CONTROL_NODE__STATE_ESTIMATOR_HPP_
#define CONTROL_NODE__STATE_ESTIMATOR_HPP_

#include "control_node/types.hpp"

namespace control_node
{

class StateEstimator
{
public:
  StateEstimator() = default;

  // 내부 상태 초기화 (모드 전환 시)
  void reset();

  // 한 프레임 갱신.
  //   robot       : 로봇 글로벌 포즈 (오도메트리). valid=false면 추정 불가.
  //   theta_head  : 상단 yaw 스테이지 각 [rad] (스텝모터 펄스 누적, 정확)
  //   owner       : 카메라가 본 주인 (azimuth/distance, is_detected)
  // 반환: 추정이 갱신/유지되어 유효하면 true.
  bool update(const RobotOdom & robot, double theta_head, const OwnerState & owner);

  // 유효한 주인 글로벌 위치 추정을 들고 있는가
  bool hasOwner() const;

  // 추정된 주인의 글로벌 위치
  Vec2 ownerGlobal() const;

private:
  bool have_owner_ = false;
  Vec2 owner_g_;     // 주인의 글로벌 위치 추정
};

}  // namespace control_node

#endif  // CONTROL_NODE__STATE_ESTIMATOR_HPP_

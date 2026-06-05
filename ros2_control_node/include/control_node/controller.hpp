// ============================================================================
//  controller.hpp  —  제어기 인터페이스 + 입력 묶음
//
//  IController : 모든 주행모드 제어기가 따르는 공통 인터페이스(전략 패턴).
//    구현체는 ControllerBase(공통 부품)를 상속한다.
//    FollowController(모드1), RotateController(모드2), ...
//
//  ※ 튜닝 파라미터(ControllerParams)는 params.hpp 로 이동(한곳 모음).
// ============================================================================
#ifndef CONTROL_NODE__CONTROLLER_HPP_
#define CONTROL_NODE__CONTROLLER_HPP_

#include "control_node/params.hpp"
#include "control_node/types.hpp"

namespace control_node
{

// ----------------------------------------------------------------------------
//  ControlInput : 제어기 step() 한 번에 필요한 모든 입력 묶음.
//   묶음으로 받으면 모드가 늘어 입력이 추가돼도 시그니처가 안 바뀐다.
// ----------------------------------------------------------------------------
struct ControlInput
{
  OwnerState owner;        // 카메라가 본 주인 (azimuth/distance 등)
  RobotOdom  robot;        // 휠 오도메트리 로봇 글로벌 포즈
  Vec2       owner_global; // StateEstimator가 추정한 주인 글로벌 위치
  bool       owner_global_valid = false;
  double     theta_head = 0.0;   // 상단 yaw 스테이지 현재 각 [rad]
  UserAdjust adjust;             // 손동작 조정값 (헤딩 오프셋/리프트 등)
  bool       hold_body = false;  // true=손동작 세션 중 → 몸체만 정지
                                 //  (상단 yaw 락온·추정은 계속. /gesture_active)
  double     dt = 0.02;          // 경과시간 [s]
};

// ----------------------------------------------------------------------------
//  IController : 제어기 공통 인터페이스 (순수가상 — 정의 없음)
// ----------------------------------------------------------------------------
class IController
{
public:
  virtual ~IController() = default;

  // 파라미터 적용(초기화·재설정 시)
  virtual void configure(const ControllerParams & params) = 0;

  // 내부 상태 초기화(모드 전환·주인 분실 시)
  virtual void reset() = 0;

  // 모드 진입 시 1회 호출: 현재 상태를 "유지할 기준"으로 캡처.
  //  (선분 유지: 지금 주인-로봇 선분의 거리·글로벌 각도를 목표로 고정)
  virtual void engage(const ControlInput & in) = 0;

  // 한 스텝 제어. 반환: 6자유도 명령(아직 장애물 보정 전).
  virtual ControlCommand step(const ControlInput & in) = 0;
};

}  // namespace control_node

#endif  // CONTROL_NODE__CONTROLLER_HPP_

// ============================================================================
//  idle_controller.hpp  —  모드0(IDLE): 자율주행 정지 + 키보드 teleop (선언)
//
//  초기/대기 모드. 자율 추적은 하지 않는다(주인 기준 동작 없음).
//   - 몸체   : /teleop_cmd(키보드)가 준 목표속도로 주행. 없으면 정지.
//              방향키 = vx/vy(대각선 동시 가능), a/d = yaw 회전.
//   - 상단yaw: 정지(현 위치 유지) — 추적 안 함.
//   - 리프트 : 손동작 리프트 명령은 여기서도 반영(applyLift).
//   - 손동작 : 메뉴 열림(hold_body) 시 몸체 정지. 모드 변경은 노드가 처리.
//
//  requiresOwner()=false → 부팅 직후 추적이 서기 전에도 즉시 teleop 가능.
//  안전: teleop도 writeBodyCommand(가속 제한) + 발행 직전 최종 클램프를 거친다.
//
//  (정의: src/idle_controller.cpp / 공통: controller_base.hpp)
// ============================================================================
#ifndef CONTROL_NODE__IDLE_CONTROLLER_HPP_
#define CONTROL_NODE__IDLE_CONTROLLER_HPP_

#include "control_node/controller_base.hpp"

namespace control_node
{

class IdleController : public ControllerBase
{
public:
  IdleController() = default;

  bool requiresOwner() const override { return false; }   // teleop은 추적 불필요

  void engage(const ControlInput & in) override;
  ControlCommand step(const ControlInput & in) override;
};

}  // namespace control_node

#endif  // CONTROL_NODE__IDLE_CONTROLLER_HPP_

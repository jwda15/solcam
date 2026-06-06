// ============================================================================
//  idle_controller.cpp  —  모드0(IDLE) 정의
//  (개념/설계 설명: include/control_node/idle_controller.hpp)
// ============================================================================
#include "control_node/idle_controller.hpp"

namespace control_node
{

void IdleController::engage(const ControlInput & in)
{
  // 상단 yaw 목표각을 현재 각으로 맞춰 둠(혹시 active로 바뀌어도 튀지 않게).
  engageCommon(in);
}

ControlCommand IdleController::step(const ControlInput & in)
{
  ControlCommand cmd;

  // ----- 상단 yaw: 정지 (추적 안 함, 현 위치 유지) -----
  //  active=false 면 드라이버가 현 위치를 그대로 잡고 있는다.
  cmd.top_yaw_active = false;
  cmd.top_yaw_target = in.theta_head;

  // ----- 리프트: 손동작 명령이 있으면 반영 (없으면 현 위치 유지) -----
  applyLift(in, cmd);

  // ----- 몸체: 손동작 메뉴 중이면 정지, 아니면 teleop 추종 -----
  if (in.hold_body) {
    stopBodySmooth(cmd, in.dt);   // 메뉴 열림 → 부드럽게 정지
    return cmd;
  }
  // teleop 목표속도를 가속 제한 걸어 출력 (없으면 0 → 자연 정지).
  //  v_max/w_body_max 최종 클램프는 노드 발행부에서 한 번 더 적용된다.
  writeBodyCommand(cmd, in.teleop_vx, in.teleop_vy, in.teleop_wz, in.dt);
  return cmd;
}

}  // namespace control_node

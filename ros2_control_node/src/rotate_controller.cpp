// ============================================================================
//  rotate_controller.cpp  —  모드2(ROTATE) "제자리 회전 추적" 정의
//  (개념/설계 설명: include/control_node/rotate_controller.hpp)
// ============================================================================
#include "control_node/rotate_controller.hpp"

namespace control_node
{

void RotateController::engage(const ControlInput & in)
{
  engageCommon(in);   // 상단 yaw 목표각 = 현재 스테이지 각
}

ControlCommand RotateController::step(const ControlInput & in)
{
  ControlCommand cmd;

  // ----- 상단 yaw(OAK-D 주인 락온) + 리프트 (공통 부품) -----
  trackTopYaw(in, cmd);
  applyLift(in, cmd);

  // 정지 처리 (상단 yaw 락온은 유지 — 정지 중에도 주인/사용자를 계속 본다):
  //  추정/오도 끊김 → 즉시 정지(안전) / 손동작 hold → 감속 정지(부드럽게)
  if (!in.owner_global_valid || !in.robot.valid) {
    stopBodyAndClearSlew(cmd);
    return cmd;
  }
  if (in.hold_body) {
    stopBodySmooth(cmd, in.dt);
    return cmd;
  }

  // ----- 몸체 yaw: 주인 방위 + 헤딩 오프셋 추종. 위치는 고정(vx=vy=0) -----
  //  주인이 코앞(방위각 특이점)이면 현 헤딩 유지(wz=0).
  double wz = 0.0;
  if (!ownerTooClose(in)) {
    wz = yawRateToHeading(
      wrapAngle(ownerBearing(in) + in.adjust.heading_offset), in);
  }

  writeBodyCommand(cmd, 0.0, 0.0, wz, in.dt);
  return cmd;
}

}  // namespace control_node

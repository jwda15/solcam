// ============================================================================
//  rotate_controller.hpp  —  모드2(ROTATE): "제자리 회전 추적" 제어기 (선언)
//
//  개념:
//   차체 위치는 고정(vx=vy=0), 메카넘 휠로 몸체 yaw 회전만 해서 주인을
//   따라 돈다. 상단 yaw(OAK-D)도 주인 락온 유지 → 정상 상태에서 촬영
//   카메라(몸체 정면)와 OAK-D가 같은 방향(주인)을 본다.
//
//  두 출력 (+리프트):
//   1) 몸체 yaw  : 주인 글로벌 방위 + heading_offset 을 추종 (P제어)
//   2) 상단 yaw  : azimuth → 0 (베이스 trackTopYaw, 모드1과 동일)
//   - 몸체 vx/vy 는 항상 0. ObstacleField 보정도 자연히 무의미(이동 없음).
//
//  heading_offset(손동작 조정)이 들어오면 촬영 카메라만 주인에서 비켜
//  보고, OAK-D는 계속 주인을 추적한다(모드1과 동일한 공유 설정).
//
//  (정의: src/rotate_controller.cpp / 게인: 몸체 yaw 계열(kp_byaw 등) 재사용)
// ============================================================================
#ifndef CONTROL_NODE__ROTATE_CONTROLLER_HPP_
#define CONTROL_NODE__ROTATE_CONTROLLER_HPP_

#include "control_node/controller_base.hpp"

namespace control_node
{

class RotateController : public ControllerBase
{
public:
  RotateController() = default;

  // 모드 진입 1회: 상단 yaw 목표각 초기화 (캡처할 기준은 없음)
  void engage(const ControlInput & in) override;

  // 한 스텝 제어: 몸체는 회전만, 위치는 고정
  ControlCommand step(const ControlInput & in) override;
};

}  // namespace control_node

#endif  // CONTROL_NODE__ROTATE_CONTROLLER_HPP_

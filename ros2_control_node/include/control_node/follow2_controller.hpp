// ============================================================================
//  follow2_controller.hpp  —  모드3(FOLLOW2): "leash(줄) 추종" 제어기 (선언)
//
//  개념:
//   주인과의 "거리"만 유지한다. 구도(글로벌 위치·선분각)는 신경 쓰지 않는다.
//   마치 주인과 줄로 묶인 것처럼 — 멀어지면(거리+데드존 초과) 끌려가고,
//   가까우면 줄이 늘어져 가만히 있는다(★후퇴 없음, 단방향). "대충 따라오기".
//
//  특징:
//   - ★/odom 불필요(requiresOwner=false): 글로벌 위치 추정 없이, 몸체 기준
//     주인 방향(theta_head - azimuth)과 거리(distance)만으로 제어.
//   - 몸체 yaw: 진입 때의 '주인 기준 상대각(rel0)'을 유지하도록 회전. 즉
//     구도(위치)는 안 잡지만, 진입 시 주인을 향하던 자세면 계속 주인을 향하고
//     측면을 보이던 자세면 계속 같은 면을 보인다(몸체장착 촬영카메라 기준).
//     상단 yaw(OAK-D)는 별도로 주인 락온 유지.
//   - 거리 데드존(leash_dead) 안이면 정지. 밖이면 밴드 끝 기준 P제어로 접근/후퇴.
//
//  게인/거리: params.hpp 의 leash_distance / leash_dead / kp_leash.
//  (정의: src/follow2_controller.cpp)
// ============================================================================
#ifndef CONTROL_NODE__FOLLOW2_CONTROLLER_HPP_
#define CONTROL_NODE__FOLLOW2_CONTROLLER_HPP_

#include "control_node/controller_base.hpp"

namespace control_node
{

class Follow2Controller : public ControllerBase
{
public:
  Follow2Controller() = default;

  // 이 모드는 글로벌 추정(odom)이 필요 없다 → 주인만 보이면 바로 동작.
  bool requiresOwner() const override { return false; }

  void engage(const ControlInput & in) override;
  ControlCommand step(const ControlInput & in) override;

  // 손동작 거리 조정 (모드1 SEG_DISTANCE 와 공유 라우팅)
  void setLeashDistance(double value, bool delta);
  double leashDistance() const { return leash_distance_; }

private:
  void onConfigure() override;     // 파라미터에서 leash_distance_ 초기화
  double leash_distance_ = 1.5;    // m, 유지할 목표 거리
  double rel0_ = 0.0;              // rad, 진입 때 캡처한 주인 기준 몸체 상대각
  bool   rel_captured_ = false;    // 진입/첫 유효 프레임에서 rel0_ 캡처 여부
};

}  // namespace control_node

#endif  // CONTROL_NODE__FOLLOW2_CONTROLLER_HPP_

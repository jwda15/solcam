// ============================================================================
//  orbit_controller.hpp  —  모드4(ORBIT): "공전(궤도) 촬영" 제어기 (선언)
//
//  개념:
//   주인을 중심으로 일정 반지름을 유지하며 주변을 천천히 돈다(드론 공전샷).
//   메카넘 게걸음으로 접선 이동 + 반지름 유지, 상단 yaw(카메라)가 주인을
//   계속 락온 → 도는 내내 주인이 화면 중앙.
//
//  특징:
//   - ★/odom 불필요(requiresOwner=false): 몸체기준 주인방향(theta_head-azimuth)
//     + 거리(distance)만으로 접선/반지름 제어.
//   - 반지름 = 모드 진입(선택) 시점의 주인 거리(engage 캡처). 손동작으로 조정.
//   - 기본 반시계(CCW), 천천히(orbit_speed). 몸체 yaw 제어 없음(wz=0).
//
//  게인/속도: params.hpp 의 orbit_speed / orbit_ccw / kp_orbit_r.
//  (정의: src/orbit_controller.cpp)
// ============================================================================
#ifndef CONTROL_NODE__ORBIT_CONTROLLER_HPP_
#define CONTROL_NODE__ORBIT_CONTROLLER_HPP_

#include "control_node/controller_base.hpp"

namespace control_node
{

class OrbitController : public ControllerBase
{
public:
  OrbitController() = default;

  bool requiresOwner() const override { return false; }   // 상대제어, odom 불필요

  void engage(const ControlInput & in) override;
  ControlCommand step(const ControlInput & in) override;

  // 손동작 반지름 조정 (모드1·3 거리조정과 공유 라우팅)
  void setOrbitRadius(double value, bool delta);
  double orbitRadius() const { return orbit_radius_; }

private:
  void onConfigure() override;     // 파라미터에서 폴백 반지름 초기화
  double orbit_radius_ = 1.5;      // m, 유지할 공전 반지름 (진입 시 캡처)
  bool   captured_ = false;        // 진입 후 첫 주인 프레임에서 반지름 캡처
};

}  // namespace control_node

#endif  // CONTROL_NODE__ORBIT_CONTROLLER_HPP_

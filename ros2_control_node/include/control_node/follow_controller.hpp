// ============================================================================
//  follow_controller.hpp  —  모드1(FOLLOW): "선분 유지" 제어기 (선언)
//
//  개념 (제어로직 대화 2026-06-03~04 확정):
//   주인-로봇을 잇는 선분의 "길이 D"와 "글로벌 각도 φ"를 모드 진입 순간 캡처해
//   계속 유지한다. 주인이 움직이면 선분이 평행이동하듯 로봇이 따라간다.
//   주인의 yaw(방향)는 쓰지 않는다 — 주인은 글로벌 평면의 점일 뿐.
//
//  목표 로봇 위치(글로벌):
//    target = owner_global - D * (cos φ, sin φ)
//    → "주인으로부터 φ 반대방향으로 D만큼 떨어진 점"
//
//  세 출력:
//   1) 몸체 vx/vy : target 으로 holonomic 이동 (글로벌 PD → 몸체프레임 회전변환)
//   2) 몸체 yaw   : 주인 방향 + heading_offset 헤딩 유지(PD, offset=0이면 정면)
//   3) 상단 yaw   : azimuth 줄이도록 스테이지 목표각 갱신 (베이스 trackTopYaw)
//
//  몸체 제어 = PD + 데드존 + 가속(슬루)제한 3단:
//   P가 끌고, D가 접근 속도를 보고 미리 감속(오버슈트 억제),
//   데드존이 도착 후 떨림 방지, 슬루가 급가감속을 차단.
//
//  손동작 조정 (/adjust_cmd → ControlNode 가 라우팅):
//   - setSegDistance()/setSegAngle() : 유지할 선분 D·φ 변경
//   - heading_offset (ControlInput.adjust) : 촬영 카메라를 주인 외 방향으로
//
//  장애물 회피로 잠깐 선분이 틀어져도 target 은 그대로라, 회피가 풀리면
//  위치오차가 살아나 자연 복귀한다(별도 복귀 로직 불필요).
//
//  (정의: src/follow_controller.cpp / 게인: params.hpp / 공통: controller_base.hpp)
// ============================================================================
#ifndef CONTROL_NODE__FOLLOW_CONTROLLER_HPP_
#define CONTROL_NODE__FOLLOW_CONTROLLER_HPP_

#include "control_node/controller_base.hpp"

namespace control_node
{

class FollowController : public ControllerBase
{
public:
  FollowController() = default;

  // 모드 진입 1회: 현재 주인-로봇 선분(거리 D, 글로벌 각도 φ)을 목표로 고정.
  void engage(const ControlInput & in) override;

  // 한 스텝 제어 (흐름: 상단yaw/리프트 → 몸체 vx/vy → 몸체 yaw → 슬루 제한)
  ControlCommand step(const ControlInput & in) override;

  // ----- 손동작 조정 (AdjustCmd 라우팅; delta=true면 증분) -----
  void setSegDistance(double value, bool delta);
  void setSegAngle(double value, bool delta);

  // ----- 디버그/모니터링용 접근자 -----
  double segDistance() const;
  double segAngle() const;

private:
  // 추가 리셋: 선분 캡처 무효화 + 위치 PD 이력 초기화
  void onReset() override;

  // 추가 설정: 위치 PD(x/y축) 게인 세팅
  void onConfigure() override;

  // 몸체 vx/vy: 선분 끝점(target) 추종 속도 계산 (글로벌 PD → 몸체 프레임)
  void computeBodyVelocity(const ControlInput & in,
                           double & vx_b, double & vy_b);

  // 유지할 선분 (모드 진입 시 캡처)
  double seg_distance_ = 1.5;   // m
  double seg_angle_    = 0.0;   // rad, 로봇→주인 글로벌 방향 φ
  bool   engaged_      = false;

  // 위치 PD (글로벌 x/y축 각각; ki=0 → PD)
  Pid pid_x_;
  Pid pid_y_;
};

}  // namespace control_node

#endif  // CONTROL_NODE__FOLLOW_CONTROLLER_HPP_

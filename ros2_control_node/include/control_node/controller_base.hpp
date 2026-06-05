// ============================================================================
//  controller_base.hpp  —  모드 제어기 공통 베이스 (선언)
//
//  여러 모드가 똑같이 쓰는 동작을 한곳에 모았다:
//   - trackTopYaw()      : 상단 yaw(OAK-D)를 주인에 락온 (azimuth → 0)
//   - applyLift()        : 손동작 리프트 목표 반영 (없으면 현 위치 유지)
//   - ownerBearing()     : 로봇→주인 글로벌 방위각
//   - yawRateToHeading() : 목표 헤딩 추종 각속도 (★PD제어 + 불감대 + 상한)
//   - writeBodyCommand() : 슬루레이트(가속) 제한 걸어 몸체 명령 기록
//
//  제어 구조(발표용 한 줄): 상위 = PD + 데드존 + 가속제한(이 노드, 속도 명령)
//                          하위 = 휠 속도 PID(팀원 드라이버, 엔코더 피드백)
//
//  새 모드 추가 절차 (확장 지점):
//   1) 이 클래스를 상속해 engage()/step() 구현 (예: rotate_controller.hpp)
//   2) ControlNode::controllerFor() 의 switch 에 한 줄 추가
//   3) 필요 파라미터는 params.hpp 에 추가
//
//  (정의: src/controller_base.cpp)
// ============================================================================
#ifndef CONTROL_NODE__CONTROLLER_BASE_HPP_
#define CONTROL_NODE__CONTROLLER_BASE_HPP_

#include "control_node/controller.hpp"
#include "control_node/pid.hpp"

namespace control_node
{

class ControllerBase : public IController
{
public:
  // 파라미터 적용 / 공통 상태 리셋 (파생은 onReset 훅으로 추가 리셋)
  void configure(const ControllerParams & params) override;
  void reset() override;

  // 디버그/모니터링: 현재 상단 yaw 목표각
  double topYawTarget() const;

protected:
  // 파생 클래스 추가 리셋 훅 (reset() 끝에 호출됨)
  virtual void onReset();

  // 파생 클래스 추가 설정 훅 (configure() 끝에 호출됨; 자체 PID 세팅용)
  virtual void onConfigure();

  // 모드 진입 공통 처리: 상단 yaw 목표각을 현재 스테이지 각으로 초기화.
  //  (스텝모터가 갑자기 크게 도는 것 방지 — 항상 현재 각에서 출발)
  void engageCommon(const ControlInput & in);

  // 상단 yaw: azimuth → 0 이 되도록 목표각을 증분 갱신 (스텝 위치 명령).
  //  주인이 우측(azimuth>0)이면 목표각 감소(CW). 변화율은 w_top_max 제한.
  void trackTopYaw(const ControlInput & in, ControlCommand & cmd);

  // 리프트: 손동작 명령(adjust.lift_commanded)이 있으면 z_min~z_max 로
  //  클램프해 위치 명령. 없으면 비활성(드라이버가 현 위치 유지).
  void applyLift(const ControlInput & in, ControlCommand & cmd) const;

  // 로봇→주인 글로벌 방위각 [rad] (owner_global_valid 전제)
  double ownerBearing(const ControlInput & in) const;

  // 주인이 로봇에 너무 가까우면(≈바로 위) 방위각이 수치적으로 무의미.
  //  true면 호출부는 yaw 명령을 0으로(헤딩 유지) 두는 것을 권장.
  bool ownerTooClose(const ControlInput & in, double min_dist = 0.15) const;

  // 목표 헤딩(desired_yaw)을 추종하는 몸체 각속도.
  //  PD제어(kp_byaw/kd_byaw) + 불감대(byaw_dead) + 상한(w_body_max).
  //  D항이 접근 속도를 보고 미리 감속 → 목표 헤딩 오버슈트 억제.
  double yawRateToHeading(double desired_yaw, const ControlInput & in);

  // 몸체 명령 기록: 가속(슬루레이트) 제한을 걸고 직전값을 갱신
  void writeBodyCommand(ControlCommand & cmd,
                        double vx, double vy, double wz, double dt);

  // 몸체 즉시 정지 + 슬루 직전값 초기화. ★안전용(추정/오도 끊김):
  //  로봇이 "눈을 감은" 상태라 감속 여유 없이 바로 멈춘다.
  void stopBodyAndClearSlew(ControlCommand & cmd);

  // 몸체 감속 정지: 목표 0으로 슬루 → body/yaw_accel_max 한계로 부드럽게.
  //  ★손동작 hold 등 "계획된 정지"용 (센서는 살아있을 때).
  void stopBodySmooth(ControlCommand & cmd, double dt);

  // 슬루레이트 제한: 한 스텝에서 target 으로 max_rate*dt 만큼만 이동
  static double slew(double current, double target, double max_rate, double dt);

  // 상단 yaw 누적각 소프트리미트(언와인딩 대비). 지금은 사실상 무한.
  double applyTopSoftLimit(double target) const;

  ControllerParams params_;
  double top_yaw_target_ = 0.0;   // rad, 상단 yaw 스테이지 목표각
  Pid    yaw_pid_;                // 몸체 yaw PD (configure에서 게인 세팅)

private:
  // 슬루용 직전 출력 (writeBodyCommand/stopBodyAndClearSlew 로만 접근)
  double prev_vx_ = 0.0, prev_vy_ = 0.0, prev_wz_ = 0.0;
};

}  // namespace control_node

#endif  // CONTROL_NODE__CONTROLLER_BASE_HPP_

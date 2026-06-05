// ============================================================================
//  pid.hpp  —  재사용 가능한 PID 제어기 + 슬루레이트 제한기 (선언)
//
//  제어 노드의 여러 축이 공통으로 쓰는 작은 부품. ROS 의존성 없음.
//  ★사용처: 몸체 yaw PD(ControllerBase), 몸체 위치 PD(FollowController).
//    ki=0 으로 두면 PD 제어기가 된다 (현재 기본 사용 방식).
//  (정의: src/pid.cpp)
// ============================================================================
#ifndef CONTROL_NODE__PID_HPP_
#define CONTROL_NODE__PID_HPP_

namespace control_node
{

// ----------------------------------------------------------------------------
//  Pid : 단일 축 PID 제어기
//
//  특징:
//   - deadzone: |오차| 가 작으면 출력 0 + 적분 리셋 (떨림 방지)
//   - 적분 windup 방지(i_clamp)
//   - 미분항은 "오차의 변화율"로 계산
//   - 출력 클램프(out_min/out_max)
//
//  사용:
//   Pid yaw_pid;
//   yaw_pid.setGains(kp, ki, kd);
//   yaw_pid.setLimits(-max, max);
//   yaw_pid.setDeadzone(0.03);
//   double u = yaw_pid.update(error, dt);
// ----------------------------------------------------------------------------
class Pid
{
public:
  Pid() = default;

  // 게인 설정
  void setGains(double kp, double ki, double kd);

  // 출력 한계 (비대칭 가능: 예) 전진 0.4, 후진 -0.2)
  void setLimits(double out_min, double out_max);

  // 적분항 절대값 상한 (windup 방지)
  void setIClamp(double i_clamp);

  // 불감대: |오차| < deadzone 이면 출력 0
  void setDeadzone(double deadzone);

  // 내부 상태 초기화 (모드 전환·주인 분실 시 호출)
  void reset();

  // 한 스텝 제어량 계산
  //   error : 목표 - 현재 (또는 정의된 오차)
  //   dt    : 경과시간 [s] (호출부에서 0/이상값 보호해서 넘길 것)
  double update(double error, double dt);

private:
  double kp_ = 0.0, ki_ = 0.0, kd_ = 0.0;
  double out_min_ = -1.0, out_max_ = 1.0;
  double i_clamp_ = 1.0;
  double deadzone_ = 0.0;

  double integral_ = 0.0;
  double prev_error_ = 0.0;
  bool   has_prev_ = false;
};

// ----------------------------------------------------------------------------
//  SlewRateLimiter : 출력 변화율(가속도) 제한
//
//  메카넘 휠 미끄럼·리프트 급발진을 막기 위해, 한 스텝에서 값이 바뀔 수 있는
//  양을 max_rate * dt 로 제한한다. 모드 전환 시 reset()으로 현재값을 맞춘다.
// ----------------------------------------------------------------------------
class SlewRateLimiter
{
public:
  SlewRateLimiter() = default;

  // max_rate : 초당 허용 변화량 (예: 0.8 m/s^2 또는 2.0 rad/s^2)
  void setMaxRate(double max_rate);

  // 현재값 리셋 (모드 전환 시 실제 출력값과 동기화)
  void reset(double value = 0.0);

  // target 으로 max_rate*dt 만큼만 이동한 값을 반환·유지
  double update(double target, double dt);

private:
  double max_rate_ = 0.0;   // 0 이면 제한 없음
  double current_ = 0.0;
};

}  // namespace control_node

#endif  // CONTROL_NODE__PID_HPP_

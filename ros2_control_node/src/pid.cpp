// ============================================================================
//  pid.cpp  —  Pid / SlewRateLimiter 정의 (선언: include/control_node/pid.hpp)
// ============================================================================
#include "control_node/pid.hpp"

#include <algorithm>
#include <cmath>

namespace control_node
{

// ============================================================================
//  Pid
// ============================================================================
void Pid::setGains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void Pid::setLimits(double out_min, double out_max)
{
  out_min_ = out_min;
  out_max_ = out_max;
}

void Pid::setIClamp(double i_clamp)
{
  i_clamp_ = i_clamp;
}

void Pid::setDeadzone(double deadzone)
{
  deadzone_ = deadzone;
}

void Pid::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  has_prev_ = false;
}

double Pid::update(double error, double dt)
{
  // 불감대: 작은 오차는 무시하고 적분 리셋(드리프트·떨림 방지)
  if (std::abs(error) < deadzone_) {
    integral_ = 0.0;
    prev_error_ = error;
    has_prev_ = true;
    return 0.0;
  }

  // 적분 (windup 클램프)
  integral_ += error * dt;
  integral_ = std::clamp(integral_, -i_clamp_, i_clamp_);

  // 미분 (첫 호출은 0으로 — 튐 방지)
  double derivative = 0.0;
  if (has_prev_ && dt > 0.0) {
    derivative = (error - prev_error_) / dt;
  }
  prev_error_ = error;
  has_prev_ = true;

  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  return std::clamp(output, out_min_, out_max_);
}

// ============================================================================
//  SlewRateLimiter
// ============================================================================
void SlewRateLimiter::setMaxRate(double max_rate)
{
  max_rate_ = max_rate;
}

void SlewRateLimiter::reset(double value)
{
  current_ = value;
}

double SlewRateLimiter::update(double target, double dt)
{
  if (dt <= 0.0 || max_rate_ <= 0.0) {
    current_ = target;
    return current_;
  }
  double max_step = max_rate_ * dt;
  double diff = target - current_;
  diff = std::clamp(diff, -max_step, max_step);
  current_ += diff;
  return current_;
}

}  // namespace control_node

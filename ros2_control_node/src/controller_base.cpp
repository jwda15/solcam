// ============================================================================
//  controller_base.cpp  —  모드 제어기 공통 베이스 정의
//  (선언/설계 설명: include/control_node/controller_base.hpp)
// ============================================================================
#include "control_node/controller_base.hpp"

#include <algorithm>
#include <cmath>

namespace control_node
{

void ControllerBase::configure(const ControllerParams & params)
{
  params_ = params;

  // 몸체 yaw PD 세팅 (ki=0 → PD). 불감대·상한은 Pid 내부에서 처리.
  yaw_pid_.setGains(params_.kp_byaw, 0.0, params_.kd_byaw);
  yaw_pid_.setLimits(-params_.w_body_max, params_.w_body_max);
  yaw_pid_.setDeadzone(params_.byaw_dead);

  onConfigure();   // 파생 클래스 자체 PID 세팅
}

void ControllerBase::reset()
{
  prev_vx_ = prev_vy_ = prev_wz_ = 0.0;
  yaw_pid_.reset();
  onReset();   // 파생 클래스 추가 리셋
}

double ControllerBase::topYawTarget() const
{
  return top_yaw_target_;
}

void ControllerBase::onReset()
{
  // 기본은 할 일 없음. 파생 클래스가 필요 시 오버라이드.
}

void ControllerBase::onConfigure()
{
  // 기본은 할 일 없음. 파생 클래스가 필요 시 오버라이드.
}

void ControllerBase::engageCommon(const ControlInput & in)
{
  top_yaw_target_ = in.theta_head;   // 현재 스테이지 각에서 출발
}

void ControllerBase::trackTopYaw(const ControlInput & in, ControlCommand & cmd)
{
  cmd.top_yaw_active = true;
  if (in.owner.is_detected && std::abs(in.owner.azimuth) > params_.az_dead) {
    double delta = -params_.kp_yaw * in.owner.azimuth;   // 목표각 증분 희망
    double max_step = params_.w_top_max * in.dt;         // 이번 스텝 허용 변화
    delta = std::clamp(delta, -max_step, max_step);
    top_yaw_target_ = applyTopSoftLimit(top_yaw_target_ + delta);

    // ★리드 제한: 목표각이 스테이지 실측각보다 top_lead_max 이상 앞서지
    //  못하게. 드라이버 지연/정지 시 목표만 무한히 도망가는 폭주 방지.
    if (params_.top_lead_max > 0.0) {
      top_yaw_target_ = std::clamp(top_yaw_target_,
                                   in.theta_head - params_.top_lead_max,
                                   in.theta_head + params_.top_lead_max);
    }
  }
  cmd.top_yaw_target = top_yaw_target_;
}

void ControllerBase::applyLift(const ControlInput & in, ControlCommand & cmd) const
{
  if (in.adjust.lift_commanded) {
    cmd.lift_height_target =
      std::clamp(in.adjust.lift_height, params_.z_min, params_.z_max);
    cmd.lift_active = true;
  } else {
    // 손동작 명령이 아직 없으면 드라이버가 현 위치 유지
    cmd.lift_height_target = params_.lift_default;
    cmd.lift_active = false;
  }
}

double ControllerBase::ownerBearing(const ControlInput & in) const
{
  return std::atan2(in.owner_global.y - in.robot.pose.y,
                    in.owner_global.x - in.robot.pose.x);
}

bool ControllerBase::ownerTooClose(const ControlInput & in, double min_dist) const
{
  return std::hypot(in.owner_global.x - in.robot.pose.x,
                    in.owner_global.y - in.robot.pose.y) < min_dist;
}

double ControllerBase::yawRateToHeading(double desired_yaw,
                                        const ControlInput & in)
{
  double yaw_err = wrapAngle(desired_yaw - in.robot.pose.yaw);
  // PD: kp*err + kd*d(err)/dt. 불감대/출력상한은 Pid 내부 처리.
  return yaw_pid_.update(yaw_err, in.dt);
}

void ControllerBase::writeBodyCommand(ControlCommand & cmd,
                                      double vx, double vy, double wz, double dt)
{
  cmd.body_vx       = slew(prev_vx_, vx, params_.body_accel_max, dt);
  cmd.body_vy       = slew(prev_vy_, vy, params_.body_accel_max, dt);
  cmd.body_yaw_rate = slew(prev_wz_, wz, params_.yaw_accel_max,  dt);
  prev_vx_ = cmd.body_vx;
  prev_vy_ = cmd.body_vy;
  prev_wz_ = cmd.body_yaw_rate;
}

void ControllerBase::stopBodyAndClearSlew(ControlCommand & cmd)
{
  cmd.stopBody();
  prev_vx_ = prev_vy_ = prev_wz_ = 0.0;
  yaw_pid_.reset();   // 정지 동안 오차 이력 무효 → 재개 시 D항 튐 방지
}

// 목표 (0,0,0)으로 슬루 — 한 스텝에 accel_max*dt 만큼만 줄어들어
// 몇 스텝에 걸쳐 자연 감속한다. (예: 0.4m/s, 0.8m/s² → 0.5초 정지)
void ControllerBase::stopBodySmooth(ControlCommand & cmd, double dt)
{
  writeBodyCommand(cmd, 0.0, 0.0, 0.0, dt);
  yaw_pid_.reset();   // 재개 시 D항 튐 방지
}

double ControllerBase::slew(double current, double target,
                            double max_rate, double dt)
{
  if (dt <= 0.0 || max_rate <= 0.0) { return target; }
  double max_step = max_rate * dt;
  double diff = std::clamp(target - current, -max_step, max_step);
  return current + diff;
}

double ControllerBase::applyTopSoftLimit(double target) const
{
  return std::clamp(target, -params_.theta_soft_max, params_.theta_soft_max);
}

}  // namespace control_node

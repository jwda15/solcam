// ============================================================================
//  control_node.cpp  —  ROS2 제어 노드 구현
//
//  입력:
//    /owner_pose     (ros2_tracking_node/OwnerPose)    주인 위치(점)
//    /odom           (nav_msgs/Odometry)               휠 오도메트리 로봇 포즈
//    /top_yaw_state  (std_msgs/Float32)                상단 yaw 현재 각[rad]
//    /control_mode   (std_msgs/Int32)                  주행 모드(supervisor)
//    /gesture_active (std_msgs/Bool)                   손동작 세션(몸체 일시정지)
//    /adjust_cmd     (ros2_control_node/AdjustCmd)     손동작 조정 명령
//    /proximity      (ros2_control_node/ProximityArray) 측면 근접센서
//  출력:
//    /control_cmd    (ros2_control_node/ControlCmd)    6자유도 명령
//
//  흐름: 콜백은 입력 캐시만 → 타이머(controlStep)가 StateEstimator로 주인
//        글로벌 위치 추정 → 활성 제어기.step() → ObstacleField 보정 → 발행
// ============================================================================
#include "control_node/control_node.hpp"

#include <chrono>
#include <cmath>
#include <vector>

using std::placeholders::_1;
using AdjustCmd = ros2_control_node::msg::AdjustCmd;

namespace control_node
{

ControlNode::ControlNode()
: rclcpp::Node("control_node")
{
  // ----- 파라미터: 선언(기본값=params.hpp) → 로드 → 부품 적용 -----
  declareParams();
  loadParams();

  idle_controller_.configure(params_);
  follow_controller_.configure(params_);
  rotate_controller_.configure(params_);
  follow2_controller_.configure(params_);
  orbit_controller_.configure(params_);
  engaged_ = false;
  obstacle_field_.setThresholds(obstacle_params_.stop_dist,
                                obstacle_params_.slow_dist);

  // ----- 발행 -----
  cmd_pub_ = this->create_publisher<ros2_control_node::msg::ControlCmd>(
    "/control_cmd", 10);
  debug_pub_ = this->create_publisher<ros2_control_node::msg::ControlDebug>(
    "/control_debug", 10);   // 튜닝/모니터링용 (드라이버는 사용 안 함)

  // ----- 구독 -----
  owner_sub_ = this->create_subscription<ros2_tracking_node::msg::OwnerPose>(
    "/owner_pose", 10, std::bind(&ControlNode::ownerCallback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(), std::bind(&ControlNode::odomCallback, this, _1));
  teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/teleop_cmd", 10, std::bind(&ControlNode::teleopCallback, this, _1));
  top_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/top_yaw_state", rclcpp::SensorDataQoS(), std::bind(&ControlNode::topYawCallback, this, _1));
  mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/control_mode", 10, std::bind(&ControlNode::modeCallback, this, _1));
  gesture_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/gesture_active", 10,
    std::bind(&ControlNode::gestureActiveCallback, this, _1));
  adjust_sub_ = this->create_subscription<AdjustCmd>(
    "/adjust_cmd", 10, std::bind(&ControlNode::adjustCallback, this, _1));
  proximity_sub_ = this->create_subscription<ros2_control_node::msg::ProximityArray>(
    "/proximity", rclcpp::SensorDataQoS(), std::bind(&ControlNode::proximityCallback, this, _1));

  // ----- 고정주기 제어 타이머 -----
  auto period = std::chrono::duration<double>(1.0 / node_params_.ctrl_rate);
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ControlNode::controlStep, this));

  last_owner_time_     = this->now();
  last_teleop_time_    = this->now();
  last_odom_time_      = this->now();
  last_proximity_time_ = this->now();
  last_step_time_      = this->now();
  last_lift_cmd_time_  = this->now();
  last_wheel_cmd_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "control_node 시작. rate=%.0fHz, mode=%d, seg_D=%.2fm, 회피=%s",
    node_params_.ctrl_rate, static_cast<int>(mode_), params_.seg_distance,
    obstacle_params_.enabled ? "on" : "off");
}

// ============================================================================
//  파라미터 선언 — 기본값은 params.hpp 구조체에서 가져온다 (한곳 관리).
//  yaml(config/control_params.yaml)이 있으면 그 값으로 덮어써진다.
// ============================================================================
void ControlNode::declareParams()
{
  const ControllerParams c;   // params.hpp 기본값
  const ObstacleParams   o;
  const NodeParams       n;

  // 상단 yaw (위치 명령)
  this->declare_parameter("kp_yaw",       c.kp_yaw);
  this->declare_parameter("az_dead",      c.az_dead);
  this->declare_parameter("w_top_max",    c.w_top_max);
  this->declare_parameter("top_lead_max", c.top_lead_max);
  this->declare_parameter("yaw_velocity_mode", c.yaw_velocity_mode);
  this->declare_parameter("top_yaw_sign",      c.top_yaw_sign);

  // 몸체 위치 (선분 끝점 추종, PD)
  this->declare_parameter("kp_pos",   c.kp_pos);
  this->declare_parameter("kd_pos",   c.kd_pos);
  this->declare_parameter("v_max",    c.v_max);
  this->declare_parameter("pos_dead", c.pos_dead);

  // 몸체 yaw (글로벌 헤딩, PD; 모드1·2 공용)
  this->declare_parameter("kp_byaw",    c.kp_byaw);
  this->declare_parameter("kd_byaw",    c.kd_byaw);
  this->declare_parameter("w_body_max", c.w_body_max);
  this->declare_parameter("byaw_dead",  c.byaw_dead);

  // 슬루레이트
  this->declare_parameter("body_accel_max", c.body_accel_max);
  this->declare_parameter("yaw_accel_max",  c.yaw_accel_max);

  // 선분 기본값 + 헤딩 정책
  this->declare_parameter("seg_distance", c.seg_distance);
  this->declare_parameter("face_owner",   c.face_owner);

  // 리프트
  this->declare_parameter("z_min",        c.z_min);
  this->declare_parameter("z_max",        c.z_max);
  this->declare_parameter("lift_default", c.lift_default);
  this->declare_parameter("lift_invert",  c.lift_invert);

  // 언와인딩(자리만)
  this->declare_parameter("theta_soft_max", c.theta_soft_max);

  // 장애물 회피
  this->declare_parameter("obstacle_avoidance", o.enabled);
  this->declare_parameter("obstacle_stop_dist", o.stop_dist);
  this->declare_parameter("obstacle_slow_dist", o.slow_dist);

  // 선분 손동작 조정 범위
  this->declare_parameter("seg_d_min", c.seg_d_min);
  this->declare_parameter("seg_d_max", c.seg_d_max);

  // 모드/주기/안전 (+카메라 지연 보상, 기본 0=꺼짐)
  this->declare_parameter("mode",              n.start_mode);
  this->declare_parameter("freeze_owner",      n.freeze_owner);
  this->declare_parameter("camera_latency",    n.camera_latency);
  this->declare_parameter("ctrl_rate",         n.ctrl_rate);
  this->declare_parameter("lift_cmd_timeout",  n.lift_cmd_timeout);
  this->declare_parameter("wheel_cmd_timeout", n.wheel_cmd_timeout);
  this->declare_parameter("owner_timeout",     n.owner_timeout);
  this->declare_parameter("odom_timeout",      n.odom_timeout);
  this->declare_parameter("proximity_timeout", n.proximity_timeout);
  this->declare_parameter("teleop_timeout",    n.teleop_timeout);
}

// ============================================================================
//  ROS 파라미터 → params_/obstacle_params_/node_params_ 로드
// ============================================================================
void ControlNode::loadParams()
{
  // --- ControllerParams ---
  params_.kp_yaw       = this->get_parameter("kp_yaw").as_double();
  params_.az_dead      = this->get_parameter("az_dead").as_double();
  params_.w_top_max    = this->get_parameter("w_top_max").as_double();
  params_.top_lead_max = this->get_parameter("top_lead_max").as_double();
  params_.yaw_velocity_mode = this->get_parameter("yaw_velocity_mode").as_bool();
  params_.top_yaw_sign      = this->get_parameter("top_yaw_sign").as_double();

  params_.kp_pos   = this->get_parameter("kp_pos").as_double();
  params_.kd_pos   = this->get_parameter("kd_pos").as_double();
  params_.v_max    = this->get_parameter("v_max").as_double();
  params_.pos_dead = this->get_parameter("pos_dead").as_double();

  params_.kp_byaw    = this->get_parameter("kp_byaw").as_double();
  params_.kd_byaw    = this->get_parameter("kd_byaw").as_double();
  params_.w_body_max = this->get_parameter("w_body_max").as_double();
  params_.byaw_dead  = this->get_parameter("byaw_dead").as_double();

  params_.body_accel_max = this->get_parameter("body_accel_max").as_double();
  params_.yaw_accel_max  = this->get_parameter("yaw_accel_max").as_double();

  params_.seg_distance = this->get_parameter("seg_distance").as_double();
  params_.seg_d_min    = this->get_parameter("seg_d_min").as_double();
  params_.seg_d_max    = this->get_parameter("seg_d_max").as_double();
  params_.face_owner   = this->get_parameter("face_owner").as_bool();

  params_.z_min        = this->get_parameter("z_min").as_double();
  params_.z_max        = this->get_parameter("z_max").as_double();
  params_.lift_default = this->get_parameter("lift_default").as_double();
  params_.lift_invert  = this->get_parameter("lift_invert").as_bool();

  params_.theta_soft_max = this->get_parameter("theta_soft_max").as_double();

  // --- ObstacleParams ---
  obstacle_params_.enabled   = this->get_parameter("obstacle_avoidance").as_bool();
  obstacle_params_.stop_dist = this->get_parameter("obstacle_stop_dist").as_double();
  obstacle_params_.slow_dist = this->get_parameter("obstacle_slow_dist").as_double();

  // --- NodeParams (+시작 모드) ---
  node_params_.start_mode        = static_cast<int>(this->get_parameter("mode").as_int());
  node_params_.freeze_owner      = this->get_parameter("freeze_owner").as_bool();
  node_params_.camera_latency    = this->get_parameter("camera_latency").as_double();
  node_params_.ctrl_rate         = this->get_parameter("ctrl_rate").as_double();
  node_params_.lift_cmd_timeout  = this->get_parameter("lift_cmd_timeout").as_double();
  node_params_.wheel_cmd_timeout = this->get_parameter("wheel_cmd_timeout").as_double();
  node_params_.owner_timeout     = this->get_parameter("owner_timeout").as_double();
  node_params_.odom_timeout      = this->get_parameter("odom_timeout").as_double();
  node_params_.proximity_timeout = this->get_parameter("proximity_timeout").as_double();
  node_params_.teleop_timeout     = this->get_parameter("teleop_timeout").as_double();

  mode_ = static_cast<Mode>(node_params_.start_mode);
}

// ============================================================================
//  콜백들 — 입력 캐시만 (실제 제어는 타이머에서)
// ============================================================================
void ControlNode::ownerCallback(const ros2_tracking_node::msg::OwnerPose::SharedPtr msg)
{
  owner_.is_detected = msg->is_detected;
  owner_.spatial_x   = msg->spatial_x;
  owner_.spatial_y   = msg->spatial_y;
  owner_.spatial_z   = msg->spatial_z;
  owner_.azimuth     = msg->azimuth;
  owner_.distance    = msg->distance;
  owner_.confidence  = msg->confidence;
  owner_.track_id    = msg->track_id;
  last_owner_time_   = this->now();
}

// 휠 오도메트리: 쿼터니언 yaw → 평면 포즈로 변환해 캐시.
void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_.valid = true;
  odom_.pose.x = msg->pose.pose.position.x;
  odom_.pose.y = msg->pose.pose.position.y;
  odom_.pose.yaw = yawFromQuat(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  odom_wz_ = msg->twist.twist.angular.z;   // 카메라 지연 보상용 yaw rate
  last_odom_time_ = this->now();
}

// 키보드 teleop(모드0): 몸체 프레임 목표속도 캐시. (teleop_keyboard.py 발행)
void ControlNode::teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  teleop_vx_ = msg->linear.x;
  teleop_vy_ = msg->linear.y;
  teleop_wz_ = msg->angular.z;
  last_teleop_time_ = this->now();
}

// 상단 yaw 스테이지 현재 각[rad] (스텝모터 펄스 누적값을 드라이버가 발행).
void ControlNode::topYawCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
  theta_head_ = msg->data;
}

// 모드 전환: 제어기 상태 리셋 + 즉시 정지(안전). 다음 스텝에서 engage.
//  adjust_(헤딩 오프셋/리프트)는 의도적으로 유지 — 모드 바꿔도 연출 유지.
void ControlNode::modeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  Mode new_mode = static_cast<Mode>(msg->data);
  if (new_mode != mode_) {
    if (IController * old_ctrl = controllerFor(mode_)) { old_ctrl->reset(); }
    mode_ = new_mode;
    estimator_.reset();
    engaged_ = false;   // 다음 스텝에서 새 제어기 engage
    owner_target_valid_ = false;   // 새 모드 확정 시 주인 타겟 재캡처
    publishStop();
    RCLCPP_INFO(this->get_logger(), "모드 변경: %d", msg->data);
  }
}

// 손동작 세션 플래그: supervisor가 따봉 인식 시 true, 명령 확정/취소 시 false.
//  true 인 동안 몸체만 정지(hold_body). 상단 yaw 락온·추정·발행은 계속
//  → 정지 중에도 사용자를 계속 보고, 풀리면 목표(선분 등)로 자연 복귀.
void ControlNode::gestureActiveCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (gesture_active_ != msg->data) {
    gesture_active_ = msg->data;
    RCLCPP_INFO(this->get_logger(),
      gesture_active_ ? "손동작 세션 시작 → 몸체 일시정지"
                      : "손동작 세션 종료 → 주행 재개");
  }
}

// 손동작 조정 명령 라우팅 (AdjustCmd.msg 의 param 상수 참조).
//  새 손동작 기능 추가 시: msg 에 상수 추가 + 여기 case 한 줄 (확장 지점).
void ControlNode::adjustCallback(const AdjustCmd::SharedPtr msg)
{
  // 휠 명령(거리/공전/팬)이 들어오는 동안만 메뉴 중 몸체 hold 를 풀어 수행한다.
  //  (명령이 끊기면 wheel_cmd_timeout 뒤 다시 정지 → "줄 때만 움직임")
  if (msg->param == AdjustCmd::PARAM_SEG_DISTANCE ||
      msg->param == AdjustCmd::PARAM_SEG_ANGLE ||
      msg->param == AdjustCmd::PARAM_HEADING_OFFSET) {
    last_wheel_cmd_time_ = this->now();
  }

  switch (msg->param) {
    case AdjustCmd::PARAM_SEG_DISTANCE:    // 모드1 선분 거리 D + 모드3 leash 거리
      follow_controller_.setSegDistance(msg->value, msg->delta);
      follow2_controller_.setLeashDistance(msg->value, msg->delta);
      orbit_controller_.setOrbitRadius(msg->value, msg->delta);
      break;
    case AdjustCmd::PARAM_SEG_ANGLE:       // 모드1: 선분 글로벌각 φ
      follow_controller_.setSegAngle(msg->value, msg->delta);
      break;
    case AdjustCmd::PARAM_HEADING_OFFSET:  // 공통: 촬영 카메라 헤딩 오프셋
      adjust_.heading_offset = wrapAngle(
        msg->delta ? adjust_.heading_offset + msg->value : msg->value);
      break;
    case AdjustCmd::PARAM_LIFT_HEIGHT:     // 공통: 리프트 방향(꾹 누름 시간기반)
      //  값 부호만 본다: +면 올림, -면 내림. 손동작이 들어오는 동안만 움직이고
      //  (last_lift_cmd_time_ 기준 lift_cmd_timeout), 끊기면 정지. applyLift 참조.
      adjust_.lift_dir = (msg->value >= 0.0f) ? +1 : -1;
      last_lift_cmd_time_ = this->now();
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "알 수 없는 adjust param: %d", msg->param);
      return;
  }
  RCLCPP_INFO(this->get_logger(),
    "조정: param=%d value=%.3f delta=%d", msg->param, msg->value, msg->delta);
}

// 근접센서: ObstacleField 갱신 + 수신시각 기록.
void ControlNode::proximityCallback(const ros2_control_node::msg::ProximityArray::SharedPtr msg)
{
  std::vector<double> dists(msg->distances.begin(), msg->distances.end());
  std::vector<double> dirs(msg->directions.begin(), msg->directions.end());
  obstacle_field_.update(dists, dirs);
  last_proximity_time_ = this->now();
}

// ============================================================================
//  고정주기 제어 루프 — 모든 제어 결정이 여기서 난다.
// ============================================================================
void ControlNode::controlStep()
{
  rclcpp::Time now = this->now();
  double dt = guardDt((now - last_step_time_).seconds());
  last_step_time_ = now;

  // 1) 현재 모드의 제어기 선택. 없으면(IDLE/미구현) 정지.
  IController * ctrl = controllerFor(mode_);
  if (ctrl == nullptr) {   // 미구현 모드(4 COMPOSE 등) → 정지 폴백
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "모드 %d 미구현 → 정지", static_cast<int>(mode_));
    publishStop();
    return;
  }

  // 2) 오도메트리 신선도 검사: 타임아웃이면 stale 포즈로 주인 글로벌
  //    위치가 오염되지 않게 무효화 (다음 /odom 수신 시 콜백이 재활성).
  if (odom_.valid && odomTimedOut()) { odom_.valid = false; }

  //    상단 yaw 각속도 추정 (지연 보상용; 스텝 피드백이라 미분 노이즈 적음)
  double theta_rate = 0.0;
  if (have_prev_theta_) { theta_rate = (theta_head_ - prev_theta_head_) / dt; }
  prev_theta_head_ = theta_head_;
  have_prev_theta_ = true;

  //    카메라 지연 보상 (기본 camera_latency=0 → 비활성):
  //    azimuth 는 (몸체 yaw + 상단 yaw)를 따라 변하므로, 지연 시간 동안의
  //    회전량만큼 azimuth 를 전진시켜 "지금" 값으로 근사.
  //    azimuth_now ≈ azimuth_cam + (wz_body + wz_top) × latency
  OwnerState owner_obs = owner_;
  // /owner_pose 끊기면 미탐지로 강등 → 상단yaw가 stale azimuth 를 계속 추적하거나
  //  추정이 오염되는 것을 막는다(몸체는 owner_global_valid 로 이미 정지).
  owner_obs.is_detected = owner_.is_detected && !ownerTimedOut();
  if (node_params_.camera_latency > 0.0 && owner_obs.is_detected) {
    owner_obs.azimuth = wrapAngle(
      owner_obs.azimuth +
      node_params_.camera_latency * (odom_wz_ + theta_rate));
  }

  //    StateEstimator로 주인 글로벌 위치 추정 (오도메트리+상단yaw각+카메라)
  bool est_ok = estimator_.update(odom_, theta_head_, owner_obs);

  // 3) 제어 입력 묶음 구성
  ControlInput in;
  in.owner = owner_obs;
  in.robot = odom_;
  in.owner_global = estimator_.ownerGlobal();
  in.owner_global_valid = est_ok && !ownerTimedOut() && !odomTimedOut();
  in.theta_head = theta_head_;
  // 리프트 시간기반: 최근 lift_cmd_timeout 안에 손동작 명령이 있었으면 active.
  //  손을 떼면(명령 끊김) active=false → 제어기가 lift_active=false 로 정지시킴.
  adjust_.lift_active_now =
    (now - last_lift_cmd_time_).seconds() < node_params_.lift_cmd_timeout;
  in.adjust = adjust_;
  // 손동작 세션(메뉴 열림)엔 몸체 정지. 단 휠 명령(거리/공전/팬)이 들어오는 동안
  //  (wheel_cmd_timeout 내)엔 hold 를 풀어 그 명령을 수행 → 명령 줄 때만 모터가 돈다.
  bool wheel_fresh =
    (now - last_wheel_cmd_time_).seconds() < node_params_.wheel_cmd_timeout;
  in.hold_body = gesture_active_ && !wheel_fresh;
  if (!teleopTimedOut()) {          // 키보드 teleop(모드0). 끊기면 0 → 자연 정지
    in.teleop_vx = teleop_vx_;
    in.teleop_vy = teleop_vy_;
    in.teleop_wz = teleop_wz_;
  }
  in.dt = dt;

  // 4) 모드 진입 처리: 기준을 안 잡았으면 engage(예: 모드1 선분 캡처).
  //    IDLE처럼 requiresOwner()=false면 추정 없이도 즉시 진입(teleop).
  if (!engaged_) {
    if (!ctrl->requiresOwner() || in.owner_global_valid) {
      ctrl->reset();
      ctrl->engage(in);
      engaged_ = true;
      // ★주인 타겟 스냅샷: 모드 확정 순간의 주인 글로벌 위치를 고정 타겟으로 캡처.
      if (in.owner_global_valid) {
        owner_target_ = in.owner_global;
        owner_target_valid_ = true;
      }
    } else {
      // 아직 추정이 안 섰으면 이번 스텝은 정지하고 다음 기회에 engage
      publishStop();
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "모드 %d 진입 대기: owner/odom 미확보", static_cast<int>(mode_));
      return;
    }
  }

  // ★주인 타겟 고정: engage 이후엔 캡처한 스냅샷을 주인 위치로 사용(실시간 대신).
  //  → 휠(거리/공전/팬)이 고정점 기준으로 안정적으로 동작. odom 살아있는 동안 유지.
  //   freeze_owner=false 면 기존처럼 실시간 주인 추종.
  if (node_params_.freeze_owner && owner_target_valid_) {
    in.owner_global = owner_target_;
    in.owner_global_valid = in.robot.valid && !odomTimedOut();
  }

  // 5) 제어 스텝
  ControlCommand cmd = ctrl->step(in);

  // 6) 장애물 회피: 몸체 속도만 깎음(목표는 불변 → 회피 후 자연 복귀)
  if (obstacle_params_.enabled) {
    if (proximityTimedOut()) { obstacle_field_.clear(); }
    obstacle_field_.apply(cmd.body_vx, cmd.body_vy);
  }

  // 7) 최종 안전 클램프 → 발행 (+디버그)
  //    publish() 내부에서도 클램프하지만(이중 안전), 디버그에 "실제 발행값"
  //    을 싣기 위해 여기서 먼저 적용한다 (클램프는 멱등이라 중복 무해).
  applySafetyLimits(cmd, params_.v_max, params_.w_body_max);
  publish(cmd);
  publishDebug(cmd, in);
}

// ============================================================================
//  헬퍼
// ============================================================================

// 모드 → 제어기 매핑 (전략 패턴). 모드 추가 시 여기 한 줄.
IController * ControlNode::controllerFor(Mode mode)
{
  switch (mode) {
    case Mode::IDLE:   return &idle_controller_;
    case Mode::FOLLOW: return &follow_controller_;
    case Mode::ROTATE: return &rotate_controller_;
    case Mode::FOLLOW2: return &follow2_controller_;
    case Mode::ORBIT:   return &orbit_controller_;
    default:           return nullptr;   // 미구현(3·4) → 정지 폴백
  }
}

bool ControlNode::ownerTimedOut() const
{
  return (this->now() - last_owner_time_).seconds() > node_params_.owner_timeout;
}

bool ControlNode::odomTimedOut() const
{
  return (this->now() - last_odom_time_).seconds() > node_params_.odom_timeout;
}

bool ControlNode::proximityTimedOut() const
{
  return (this->now() - last_proximity_time_).seconds() >
         node_params_.proximity_timeout;
}

bool ControlNode::teleopTimedOut() const
{
  return (this->now() - last_teleop_time_).seconds() > node_params_.teleop_timeout;
}

// 평면 yaw 만 필요하므로 z/w 로 추출 (roll/pitch 무시).
double ControlNode::yawFromQuat(double x, double y, double z, double w)
{
  double siny = 2.0 * (w * z + x * y);
  double cosy = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(siny, cosy);
}

// 안전 정지: 몸체속도 0, 스텝(리프트/상단yaw)은 현 위치 유지.
void ControlNode::publishStop()
{
  ControlCommand stop;
  stop.zero();   // body 0 + lift/top inactive(현 위치 유지)
  publish(stop);
}

// ControlCommand(내부) → ControlCmd.msg(ROS) 변환·발행.
//  발행 직전 ★최종 안전 클램프: 어떤 경로로 만들어진 명령이든
//  v_max / w_body_max 를 절대 넘지 않는다 (휠모터 토크 보호).
void ControlNode::publish(const ControlCommand & cmd)
{
  ControlCommand out = cmd;
  applySafetyLimits(out, params_.v_max, params_.w_body_max);

  ros2_control_node::msg::ControlCmd msg;
  msg.header.stamp = this->now();
  msg.body_vx            = static_cast<float>(out.body_vx);
  msg.body_vy            = static_cast<float>(out.body_vy);
  msg.body_yaw_rate      = static_cast<float>(out.body_yaw_rate);
  msg.lift_height_target = static_cast<float>(out.lift_height_target);
  msg.lift_active        = out.lift_active;
  msg.top_yaw_target     = static_cast<float>(out.top_yaw_target);
  msg.top_yaw_active     = out.top_yaw_active;
  cmd_pub_->publish(msg);
}

// 내부 상태 → /control_debug 발행 (실주행 튜닝: rqt_plot, topic echo)
void ControlNode::publishDebug(const ControlCommand & cmd, const ControlInput & in)
{
  ros2_control_node::msg::ControlDebug d;
  d.header.stamp = this->now();
  d.mode               = static_cast<int>(mode_);
  d.engaged            = engaged_;
  d.gesture_hold       = gesture_active_;
  d.owner_global_valid = in.owner_global_valid;
  d.owner_gx           = static_cast<float>(in.owner_global.x);
  d.owner_gy           = static_cast<float>(in.owner_global.y);
  d.seg_distance       = static_cast<float>(follow_controller_.segDistance());
  d.seg_angle          = static_cast<float>(follow_controller_.segAngle());
  d.heading_offset     = static_cast<float>(adjust_.heading_offset);
  d.top_yaw_target     = static_cast<float>(cmd.top_yaw_target);
  d.cmd_vx             = static_cast<float>(cmd.body_vx);
  d.cmd_vy             = static_cast<float>(cmd.body_vy);
  d.cmd_wz             = static_cast<float>(cmd.body_yaw_rate);
  debug_pub_->publish(d);
}

}  // namespace control_node

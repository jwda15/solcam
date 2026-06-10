// ============================================================================
//  control_node.hpp  —  ROS2 진입점 노드 선언
//
//  책임:
//   - 토픽 구독/발행, 파라미터 로드, 고정주기 제어 타이머
//   - 입력 캐시(주인/오도메트리/상단yaw각/근접센서) → StateEstimator로 주인
//     글로벌 위치 추정 → 활성 모드 제어기 → 장애물 보정 → ControlCmd 발행
//   - 손동작 조정(/adjust_cmd) 라우팅: 선분 조정은 FollowController로,
//     헤딩 오프셋/리프트는 공유 상태(adjust_)로 → 모드 바꿔도 유지
//   - 손동작 세션(/gesture_active): true 동안 몸체만 일시정지(hold_body)
//   - 제어 로직 자체는 제어기/부품에 위임(이 노드엔 없음)
//
//  모드 추가 시: 제어기 멤버 추가 + controllerFor() switch 한 줄.
//
//  (정의: src/control_node.cpp / 튜닝 파라미터: params.hpp + control_params.yaml)
// ============================================================================
#ifndef CONTROL_NODE__CONTROL_NODE_HPP_
#define CONTROL_NODE__CONTROL_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ros2_tracking_node/msg/owner_pose.hpp"
#include "ros2_control_node/msg/control_cmd.hpp"
#include "ros2_control_node/msg/proximity_array.hpp"
#include "ros2_control_node/msg/adjust_cmd.hpp"
#include "ros2_control_node/msg/control_debug.hpp"

#include "control_node/params.hpp"
#include "control_node/types.hpp"
#include "control_node/controller.hpp"
#include "control_node/idle_controller.hpp"
#include "control_node/follow_controller.hpp"
#include "control_node/rotate_controller.hpp"
#include "control_node/follow2_controller.hpp"
#include "control_node/state_estimator.hpp"
#include "control_node/obstacle_field.hpp"

namespace control_node
{

class ControlNode : public rclcpp::Node
{
public:
  ControlNode();

private:
  // ----- 콜백 (입력 캐시만; 제어는 타이머에서) -----
  void ownerCallback(const ros2_tracking_node::msg::OwnerPose::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void topYawCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void modeCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void gestureActiveCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void adjustCallback(const ros2_control_node::msg::AdjustCmd::SharedPtr msg);
  void proximityCallback(const ros2_control_node::msg::ProximityArray::SharedPtr msg);

  // ----- 주기 제어 루프 -----
  void controlStep();

  // ----- 헬퍼 -----
  //  현재 모드의 제어기 반환. 미구현 모드/IDLE 은 nullptr (→정지 폴백).
  IController * controllerFor(Mode mode);
  void declareParams();   // ROS 파라미터 선언 (params.hpp 기본값 사용)
  void loadParams();      // ROS 파라미터 → params_/obstacle_params_/node_params_
  void publishStop();
  void publish(const ControlCommand & cmd);
  void publishDebug(const ControlCommand & cmd, const ControlInput & in);
  bool ownerTimedOut() const;
  bool odomTimedOut() const;
  bool proximityTimedOut() const;
  bool teleopTimedOut() const;
  static double yawFromQuat(double x, double y, double z, double w);

  // ----- 파라미터 (튜닝값 정의는 전부 params.hpp) -----
  ControllerParams params_;            // 제어기 게인/한계
  ObstacleParams   obstacle_params_;   // 회피 임계값
  NodeParams       node_params_;       // 주기/타임아웃/시작모드

  // ----- 상태 -----
  Mode        mode_ = Mode::IDLE;
  OwnerState  owner_;
  RobotOdom   odom_;
  double      theta_head_ = 0.0;     // 상단 yaw 현재 각 [rad]
  double      odom_wz_ = 0.0;        // 오도메트리 측정 몸체 yaw rate [rad/s]
  double      prev_theta_head_ = 0.0;    // 상단 yaw 속도 추정용 직전값
  bool        have_prev_theta_ = false;
  UserAdjust  adjust_;               // 손동작 조정값 (모드 전환에도 유지)
  bool        gesture_active_ = false;   // 손동작 세션 중(몸체 일시정지)
  double      teleop_vx_ = 0.0, teleop_vy_ = 0.0, teleop_wz_ = 0.0;  // 키보드 teleop
  rclcpp::Time last_owner_time_;
  rclcpp::Time last_teleop_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_proximity_time_;
  rclcpp::Time last_step_time_;

  // ----- 부품 (전략 패턴: controllerFor()가 모드에 맞는 제어기 선택) -----
  IdleController   idle_controller_;     // 모드0: 정지 + 키보드 teleop
  FollowController follow_controller_;   // 모드1: 선분 유지
  RotateController rotate_controller_;   // 모드2: 제자리 회전 추적
  Follow2Controller follow2_controller_; // 모드3: leash(거리만 유지)
  StateEstimator   estimator_;
  ObstacleField    obstacle_field_;
  bool             engaged_ = false;     // 현 모드 engage(기준 캡처) 완료 여부

  // ----- ROS 인터페이스 -----
  rclcpp::Publisher<ros2_control_node::msg::ControlCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ros2_control_node::msg::ControlDebug>::SharedPtr debug_pub_;
  rclcpp::Subscription<ros2_tracking_node::msg::OwnerPose>::SharedPtr owner_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr top_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gesture_sub_;
  rclcpp::Subscription<ros2_control_node::msg::AdjustCmd>::SharedPtr adjust_sub_;
  rclcpp::Subscription<ros2_control_node::msg::ProximityArray>::SharedPtr proximity_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace control_node

#endif  // CONTROL_NODE__CONTROL_NODE_HPP_

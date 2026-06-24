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
#include "std_msgs/msg/empty.hpp"
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
#include "control_node/orbit_controller.hpp"
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
  void yawSetZeroCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void yawSetAngleCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void estopCallback(const std_msgs::msg::Bool::SharedPtr msg);  // 스페이스바 긴급정지
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
  // 상단 yaw 데드레코닝 + ±한계 방어 + 0점 쿨다운. ctrl->step() 뒤 cmd 후처리.
  //  명령부호로 head_angle_ 적분, 한계 넘으면 정지, 쿨다운 중엔 완전정지.
  void applyTopYawGuard(ControlCommand & cmd, double dt, double owner_dist,
                        double owner_az, bool owner_detected);
  double targetDistanceForMode();   // UI/디버그용: 현재 모드의 유지 목표거리[m]
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
  double      theta_head_ = 0.0;     // /top_yaw_state 피드백(있을 때만; 보통 미발행)
  double      head_angle_ = 0.0;     // ★데드레코닝 상단yaw 현재각[rad] (0=부팅/0점지정).
                                     //  제어/추정/한계판정의 실제 기준값.
  rclcpp::Time yaw_zero_block_until_; // 이 시각까지 상단yaw 정지(0점 지정 쿨다운)
  // ----- 상단 yaw 펄스 제어 + 시간기반 케이블 가드 상태 -----
  double       yaw_time_accum_ = 0.0; // 한 방향 누적 명령시간[s] (0=부팅/0점지정). 가드 기준
  rclcpp::Time yaw_pulse_until_;      // 현재 펄스가 이 시각까지 active
  rclcpp::Time yaw_last_pulse_;       // 마지막 펄스 시작 시각(주기 판정)
  int          yaw_pulse_dir_ = 0;    // 현재 펄스 방향(+1/-1)
  bool         yaw_warn_latched_ = false;  // 한계근접 경고 1회 발행 디바운스
  // ----- 촬영구도 기동(프리셋 선택 후 자동 자전+재정렬) 상태 -----
  bool         compose_active_ = false;    // 기동 중(주행 보류)
  bool         compose_pub_last_ = false;  // /compose_active 마지막 발행값(변화 시만)
  rclcpp::Time compose_until_;             // 이 시각까지 기동(회전시간 추정 + 정착 2s)
  double      odom_wz_ = 0.0;        // 오도메트리 측정 몸체 yaw rate [rad/s]
  double      prev_theta_head_ = 0.0;    // 상단 yaw 속도 추정용 직전값
  bool        have_prev_theta_ = false;
  UserAdjust  adjust_;               // 손동작 조정값 (모드 전환에도 유지)
  Vec2        owner_target_;             // 모드 확정(engage) 시 캡처한 주인 글로벌 타겟
  bool        owner_target_valid_ = false;  // 캡처 완료 여부(고정 타겟 사용)
  bool        gesture_active_ = false;   // 손동작 세션 중(몸체 일시정지)
  bool        estop_active_ = false;      // ★스페이스바 긴급정지(상단yaw·리프트 즉시, 휠은 감속)
  double      pub_vx_ = 0.0, pub_vy_ = 0.0, pub_wz_ = 0.0;  // 직전 발행 몸체속도(estop 감속 시드)
  double      teleop_vx_ = 0.0, teleop_vy_ = 0.0, teleop_wz_ = 0.0;  // 키보드 teleop
  double      jog_vx_ = 0.0, jog_vy_ = 0.0, jog_wz_ = 0.0;  // Wheel 로봇기준 jog(모든 모드)
  double      jog_orbit_ = 0.0, jog_radial_ = 0.0;  // Wheel 주인기준 jog(접선/반경, 모든 모드)
  rclcpp::Time last_owner_time_;
  rclcpp::Time last_teleop_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_proximity_time_;
  rclcpp::Time last_step_time_;
  rclcpp::Time last_lift_cmd_time_;   // 리프트 손동작 명령 마지막 수신(시간기반 제어)
  rclcpp::Time last_wheel_cmd_time_;  // 휠 명령(거리/공전/팬) 마지막 수신(메뉴 중 hold 해제용)
  rclcpp::Time last_jog_time_;        // Wheel 로봇기준 jog 마지막 수신

  // ----- 부품 (전략 패턴: controllerFor()가 모드에 맞는 제어기 선택) -----
  IdleController   idle_controller_;     // 모드0: 정지 + 키보드 teleop
  FollowController follow_controller_;   // 모드1: 선분 유지
  RotateController rotate_controller_;   // 모드2: 제자리 회전 추적
  Follow2Controller follow2_controller_; // 모드3: leash(거리만 유지)
  OrbitController   orbit_controller_;   // 모드4: 공전(반지름 유지하며 돌기)
  StateEstimator   estimator_;
  ObstacleField    obstacle_field_;
  bool             engaged_ = false;     // 현 모드 engage(기준 캡처) 완료 여부

  // ----- ROS 인터페이스 -----
  rclcpp::Publisher<ros2_control_node::msg::ControlCmd>::SharedPtr cmd_pub_;
  rclcpp::Publisher<ros2_control_node::msg::ControlDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr yaw_warn_pub_;  // 한계근접→UI 빨간선
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr compose_pub_;    // 구도기동중→UI 파란선
  rclcpp::Subscription<ros2_tracking_node::msg::OwnerPose>::SharedPtr owner_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr top_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gesture_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr yaw_zero_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<ros2_control_node::msg::AdjustCmd>::SharedPtr adjust_sub_;
  rclcpp::Subscription<ros2_control_node::msg::ProximityArray>::SharedPtr proximity_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace control_node

#endif  // CONTROL_NODE__CONTROL_NODE_HPP_

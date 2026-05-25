#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_tracking_node/msg/owner_pose.hpp"

#include <cmath>
#include <chrono>
#include <string>

// --------------------------------------------------------
// FollowNode (Jackal 차동구동 + PID 버전)
//
// 핵심 아이디어:
//   "주인이 있어야 하는 위치" = target_pose_ (카메라 좌표계 기준 3D 벡터)
//     - target_pose_.x : 좌우 (m), 양수=오른쪽
//     - target_pose_.y : 상하 (m), 양수=아래   (Jackal에선 미사용, 틸트 서보가 담당)
//     - target_pose_.z : 전방거리 (m), 양수=앞
//
//   기본값은 "화면 중앙, 거리 target_distance" → (0, 0, target_distance)
//   나중에 손동작이나 다른 인터페이스로 이 벡터만 바꿔주면 따라가는 위치가 바뀜
//
// 모드:
//   0 = 대기  : 모든 출력 0
//   1 = 팔로우: yaw + 전후 (Jackal 차동구동)
//   2 = 고정  : yaw만 (제자리 회전으로 주인 정면에 두기)
//
// 입력  : /owner_pose     (ros2_tracking_node/OwnerPose)
// 입력  : /follow_mode    (std_msgs/Int32)
// 입력  : /follow_target  (geometry_msgs/Vector3, 카메라 좌표계 m)
// 출력  : /cmd_vel        (geometry_msgs/Twist)
// --------------------------------------------------------

class FollowNode : public rclcpp::Node
{
public:
    FollowNode() : Node("follow_node")
    {
        // ----- 모드 -----
        this->declare_parameter("mode", 0);

        // ----- 목표 위치 벡터 (카메라 좌표계, m) -----
        // 기본: 화면 중앙(x=0), 높이 무관(y=0), 전방 target_distance
        this->declare_parameter("target_x", 0.0);
        this->declare_parameter("target_y", 0.0);
        this->declare_parameter("target_z", 2.0);   // 전방 2m

        // ----- PID 게인: yaw (azimuth 오차 보정) -----
        this->declare_parameter("kp_yaw", 1.2);
        this->declare_parameter("ki_yaw", 0.0);
        this->declare_parameter("kd_yaw", 0.15);

        // ----- PID 게인: forward (전방거리 오차 보정) -----
        this->declare_parameter("kp_forward", 0.6);
        this->declare_parameter("ki_forward", 0.0);
        this->declare_parameter("kd_forward", 0.10);

        // ----- 출력 제한 (Jackal 안전치) -----
        this->declare_parameter("max_yaw",     0.6);   // rad/s
        this->declare_parameter("max_forward", 0.4);   // m/s
        this->declare_parameter("max_reverse", 0.2);   // m/s (후진은 더 보수적)

        // ----- Deadzone -----
        this->declare_parameter("deadzone_azimuth",  0.04);  // rad (약 2.3도)
        this->declare_parameter("deadzone_distance", 0.10);  // m

        // ----- 적분항 windup 방지 -----
        this->declare_parameter("i_clamp_yaw",     0.5);
        this->declare_parameter("i_clamp_forward", 0.3);

        // ----- 안전: owner_pose 타임아웃 (초) -----
        this->declare_parameter("owner_timeout_sec", 0.5);

        load_params();

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        owner_sub_ = this->create_subscription<ros2_tracking_node::msg::OwnerPose>(
            "/owner_pose", 10,
            std::bind(&FollowNode::owner_callback, this, std::placeholders::_1));

        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/follow_mode", 10,
            std::bind(&FollowNode::mode_callback, this, std::placeholders::_1));

        // /follow_target: x=좌우(m), y=상하(m), z=전방(m)
        target_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/follow_target", 10,
            std::bind(&FollowNode::target_callback, this, std::placeholders::_1));

        // 안전 타이머: owner_pose가 끊기면 정지
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FollowNode::watchdog_callback, this));

        last_owner_time_ = this->now();
        last_ctrl_time_  = this->now();

        RCLCPP_INFO(this->get_logger(),
            "FollowNode 시작. 모드=%d, target=(%.2f, %.2f, %.2f)",
            mode_, target_x_, target_y_, target_z_);
    }

private:
    void load_params()
    {
        mode_     = this->get_parameter("mode").as_int();
        target_x_ = (float)this->get_parameter("target_x").as_double();
        target_y_ = (float)this->get_parameter("target_y").as_double();
        target_z_ = (float)this->get_parameter("target_z").as_double();

        kp_yaw_ = (float)this->get_parameter("kp_yaw").as_double();
        ki_yaw_ = (float)this->get_parameter("ki_yaw").as_double();
        kd_yaw_ = (float)this->get_parameter("kd_yaw").as_double();

        kp_fwd_ = (float)this->get_parameter("kp_forward").as_double();
        ki_fwd_ = (float)this->get_parameter("ki_forward").as_double();
        kd_fwd_ = (float)this->get_parameter("kd_forward").as_double();

        max_yaw_     = (float)this->get_parameter("max_yaw").as_double();
        max_forward_ = (float)this->get_parameter("max_forward").as_double();
        max_reverse_ = (float)this->get_parameter("max_reverse").as_double();

        deadzone_azimuth_  = (float)this->get_parameter("deadzone_azimuth").as_double();
        deadzone_distance_ = (float)this->get_parameter("deadzone_distance").as_double();

        i_clamp_yaw_ = (float)this->get_parameter("i_clamp_yaw").as_double();
        i_clamp_fwd_ = (float)this->get_parameter("i_clamp_forward").as_double();

        owner_timeout_sec_ = this->get_parameter("owner_timeout_sec").as_double();
    }

    void mode_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        mode_ = msg->data;
        reset_pid();
        publish_stop();
        RCLCPP_INFO(this->get_logger(), "모드 변경: %d", mode_);
    }

    void target_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        target_x_ = (float)msg->x;
        target_y_ = (float)msg->y;
        target_z_ = (float)msg->z;
        reset_pid();
        RCLCPP_INFO(this->get_logger(),
            "타겟 위치 변경: (%.2f, %.2f, %.2f) m",
            target_x_, target_y_, target_z_);
    }

    // 주인 위치 수신 → PID 제어 → /cmd_vel
    void owner_callback(const ros2_tracking_node::msg::OwnerPose::SharedPtr msg)
    {
        last_owner_time_ = this->now();

        auto twist = geometry_msgs::msg::Twist();

        if (mode_ == 0) { cmd_pub_->publish(twist); return; }

        if (!msg->is_detected) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "주인 미탐지. 정지.");
            reset_pid();
            cmd_pub_->publish(twist);
            return;
        }

        // --------------------------------------------------------
        // 오차 계산
        //   주인 현재 위치 (sx, sy, sz) vs 타겟 위치 (target_x_, target_y_, target_z_)
        //
        //   azimuth_err  : 주인을 타겟의 좌우방향에 두기 위한 yaw 오차 (rad)
        //   distance_err : 주인을 타겟의 전방거리에 두기 위한 거리 오차 (m)
        //                  +면 너무 멀다 → 전진,  -면 너무 가깝다 → 후진
        // --------------------------------------------------------
        float sx = msg->spatial_x;
        float sz = msg->spatial_z;

        // 타겟의 azimuth = atan2(target_x, target_z)
        // 주인의 azimuth = atan2(sx, sz)
        float azimuth_target = std::atan2(target_x_, target_z_);
        float azimuth_actual = std::atan2(sx, sz);
        float azimuth_err    = azimuth_actual - azimuth_target;   // 양수면 주인이 타겟보다 오른쪽

        // 거리: 단순히 z방향 오차로 처리 (소거 가능한 방위각만큼은 yaw가 처리)
        // 주인 z - 타겟 z. 양수면 주인이 멀리 있음 → 전진 필요
        float distance_err = sz - target_z_;

        // dt
        rclcpp::Time now = this->now();
        double dt = (now - last_ctrl_time_).seconds();
        last_ctrl_time_ = now;
        if (dt <= 0.0 || dt > 1.0) dt = 0.033;   // 초기/이상값 보호

        // --------------------------------------------------------
        // PID
        // --------------------------------------------------------
        // yaw 제어: yaw 양수면 좌회전 → 주인이 오른쪽(azimuth_err>0)이면 우회전(yaw 음수)
        float yaw_cmd = 0.0f;
        if (std::abs(azimuth_err) > deadzone_azimuth_) {
            i_yaw_ += azimuth_err * (float)dt;
            i_yaw_  = clamp(i_yaw_, -i_clamp_yaw_, i_clamp_yaw_);
            float d_yaw = (azimuth_err - prev_az_err_) / (float)dt;
            yaw_cmd = -(kp_yaw_ * azimuth_err + ki_yaw_ * i_yaw_ + kd_yaw_ * d_yaw);
            yaw_cmd = clamp(yaw_cmd, -max_yaw_, max_yaw_);
        } else {
            i_yaw_ = 0.0f;
        }
        prev_az_err_ = azimuth_err;

        // 전후 제어: distance_err 양수 → 전진(linear.x 양수)
        float fwd_cmd = 0.0f;
        if (mode_ == 1) {
            if (std::abs(distance_err) > deadzone_distance_) {
                i_fwd_ += distance_err * (float)dt;
                i_fwd_  = clamp(i_fwd_, -i_clamp_fwd_, i_clamp_fwd_);
                float d_fwd = (distance_err - prev_dist_err_) / (float)dt;
                fwd_cmd = kp_fwd_ * distance_err + ki_fwd_ * i_fwd_ + kd_fwd_ * d_fwd;
                fwd_cmd = clamp(fwd_cmd, -max_reverse_, max_forward_);
            } else {
                i_fwd_ = 0.0f;
            }
        }
        prev_dist_err_ = distance_err;

        twist.linear.x  = fwd_cmd;
        twist.angular.z = yaw_cmd;
        // Jackal 차동구동: linear.y, linear.z, angular.x, angular.y 는 0 유지

        cmd_pub_->publish(twist);
    }

    // owner_pose 끊기면 안전 정지
    void watchdog_callback()
    {
        double elapsed = (this->now() - last_owner_time_).seconds();
        if (mode_ != 0 && elapsed > owner_timeout_sec_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "owner_pose 타임아웃 (%.2fs). 안전 정지.", elapsed);
            reset_pid();
            publish_stop();
        }
    }

    void publish_stop()
    {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
    }

    void reset_pid()
    {
        i_yaw_         = 0.0f;
        i_fwd_         = 0.0f;
        prev_az_err_   = 0.0f;
        prev_dist_err_ = 0.0f;
    }

    static float clamp(float v, float lo, float hi)
    {
        return std::max(lo, std::min(hi, v));
    }

    // ----- 멤버 -----
    int   mode_ = 0;

    // 주인이 있어야 하는 위치 (카메라 좌표계, m)
    float target_x_ = 0.0f;
    float target_y_ = 0.0f;
    float target_z_ = 2.0f;

    // PID 게인
    float kp_yaw_ = 1.2f, ki_yaw_ = 0.0f, kd_yaw_ = 0.15f;
    float kp_fwd_ = 0.6f, ki_fwd_ = 0.0f, kd_fwd_ = 0.10f;

    // 출력 제한
    float max_yaw_ = 0.6f, max_forward_ = 0.4f, max_reverse_ = 0.2f;

    // Deadzone / windup
    float deadzone_azimuth_ = 0.04f, deadzone_distance_ = 0.10f;
    float i_clamp_yaw_ = 0.5f, i_clamp_fwd_ = 0.3f;

    // PID 상태
    float i_yaw_ = 0.0f, i_fwd_ = 0.0f;
    float prev_az_err_ = 0.0f, prev_dist_err_ = 0.0f;

    // 안전
    double owner_timeout_sec_ = 0.5;
    rclcpp::Time last_owner_time_, last_ctrl_time_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<ros2_tracking_node::msg::OwnerPose>::SharedPtr owner_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowNode>());
    rclcpp::shutdown();
    return 0;
}

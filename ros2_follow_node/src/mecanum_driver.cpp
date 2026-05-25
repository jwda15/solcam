#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <cmath>
#include <array>

// --------------------------------------------------------
// MecanumDriver: /cmd_vel → 메카넘 휠 4개 + 승강부 출력
//
// 입력: geometry_msgs/Twist
//   linear.x  → 전후 이동 (vx, m/s)
//   linear.y  → 좌우 이동 (vy, m/s) [메카넘만 가능]
//   linear.z  → 승강부 상하 (vz, m/s)
//   angular.z → yaw 회전 (wz, rad/s)
//
// 출력: std_msgs/Float32MultiArray [FL, FR, RL, RR, LIFT]
//   각 값: -1.0 ~ +1.0 (정규화된 모터 출력)
//   STM32 드라이버 노드가 이 값을 PWM으로 변환
//
// 메카넘 휠 배치 (위에서 본 그림):
//   FL ↗  FR ↘      (+x = 앞, +y = 왼쪽)
//   RL ↘  RR ↗
//
// 운동학 공식:
//   FL =  vx - vy - wz * L
//   FR =  vx + vy + wz * L
//   RL =  vx + vy - wz * L
//   RR =  vx - vy + wz * L
//   (L = 휠베이스 보정 계수)
// --------------------------------------------------------

class MecanumDriver : public rclcpp::Node
{
public:
    MecanumDriver() : Node("mecanum_driver")
    {
        // 파라미터 선언
        this->declare_parameter("wheel_base_coeff", 1.0);  // 휠베이스 보정 (L)
                                                            // (wheel_track + wheel_base) / 2
        this->declare_parameter("max_wheel_speed",  1.0);  // 최대 휠 속도 (정규화 기준)
        this->declare_parameter("max_lift_speed",   1.0);  // 최대 승강 속도 (정규화 기준)

        wheel_base_coeff_ = (float)this->get_parameter("wheel_base_coeff").as_double();
        max_wheel_speed_  = (float)this->get_parameter("max_wheel_speed").as_double();
        max_lift_speed_   = (float)this->get_parameter("max_lift_speed").as_double();

        // /cmd_vel 구독
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MecanumDriver::cmd_callback, this, std::placeholders::_1));

        // 휠 출력 publish (STM32 드라이버 노드로 전달)
        // [FL, FR, RL, RR, LIFT] 순서, -1.0 ~ +1.0
        wheel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/wheel_cmd", 10);

        RCLCPP_INFO(this->get_logger(), "MecanumDriver 시작");
        RCLCPP_INFO(this->get_logger(),
            "wheel_base_coeff=%.2f, max_wheel=%.2f, max_lift=%.2f",
            wheel_base_coeff_, max_wheel_speed_, max_lift_speed_);
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        float vx = (float)msg->linear.x;   // 전후
        float vy = (float)msg->linear.y;   // 좌우 (현재 follow_node는 0)
        float vz = (float)msg->linear.z;   // 승강
        float wz = (float)msg->angular.z;  // yaw

        // --------------------------------------------------------
        // 메카넘 휠 역운동학
        // --------------------------------------------------------
        float fl =  vx - vy - wz * wheel_base_coeff_;
        float fr =  vx + vy + wz * wheel_base_coeff_;
        float rl =  vx + vy - wz * wheel_base_coeff_;
        float rr =  vx - vy + wz * wheel_base_coeff_;

        // 최대값으로 정규화 (어느 휠도 최대값을 초과하지 않도록)
        float max_val = std::max({std::abs(fl), std::abs(fr),
                                  std::abs(rl), std::abs(rr), 1.0f});
        fl /= max_val;
        fr /= max_val;
        rl /= max_val;
        rr /= max_val;

        // max_wheel_speed_ 스케일 적용
        fl *= max_wheel_speed_;
        fr *= max_wheel_speed_;
        rl *= max_wheel_speed_;
        rr *= max_wheel_speed_;

        // 승강부 정규화
        float lift = clamp(vz / max_lift_speed_, -1.0f, 1.0f);

        // --------------------------------------------------------
        // publish: [FL, FR, RL, RR, LIFT]
        // --------------------------------------------------------
        auto out = std_msgs::msg::Float32MultiArray();
        out.data = {fl, fr, rl, rr, lift};
        wheel_pub_->publish(out);

        RCLCPP_DEBUG(this->get_logger(),
            "FL=%.2f FR=%.2f RL=%.2f RR=%.2f LIFT=%.2f",
            fl, fr, rl, rr, lift);
    }

    float clamp(float val, float min_val, float max_val)
    {
        return std::max(min_val, std::min(max_val, val));
    }

    float wheel_base_coeff_ = 1.0f;
    float max_wheel_speed_  = 1.0f;
    float max_lift_speed_   = 1.0f;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumDriver>());
    rclcpp::shutdown();
    return 0;
}

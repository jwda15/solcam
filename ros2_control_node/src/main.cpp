// ============================================================================
//  main.cpp  —  control_node 실행 진입점
// ============================================================================
#include "control_node/control_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control_node::ControlNode>());
  rclcpp::shutdown();
  return 0;
}

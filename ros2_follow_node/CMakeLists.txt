cmake_minimum_required(VERSION 3.14)
project(ros2_follow_node)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_tracking_node REQUIRED)

add_executable(follow_node src/follow_node.cpp)
ament_target_dependencies(follow_node rclcpp std_msgs geometry_msgs ros2_tracking_node)

add_executable(mecanum_driver src/mecanum_driver.cpp)
ament_target_dependencies(mecanum_driver rclcpp geometry_msgs std_msgs)

install(TARGETS follow_node mecanum_driver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

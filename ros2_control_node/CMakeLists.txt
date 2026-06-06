cmake_minimum_required(VERSION 3.14)
project(ros2_control_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------- 의존 패키지 --------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)         # Twist 사용(키보드 teleop)
find_package(nav_msgs REQUIRED)             # Odometry.msg 사용(휠 오도메트리)
find_package(rosidl_default_generators REQUIRED)
find_package(ros2_tracking_node REQUIRED)   # OwnerPose.msg 사용

# -------- 메시지 생성 (이 패키지 자체 메시지) --------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ControlCmd.msg"
  "msg/ProximityArray.msg"
  "msg/AdjustCmd.msg"
  "msg/ControlDebug.msg"
  DEPENDENCIES std_msgs
)

# -------- 노드 실행파일 --------
#  헤더(include/control_node/*.hpp)는 선언만, 정의는 src/*.cpp (가독성).
add_executable(control_node
  src/types.cpp               # 공통 타입/각도 헬퍼
  src/pid.cpp                 # PID + 슬루 (타 모드용, 현재 미사용)
  src/obstacle_field.cpp      # 측면센서 간단 회피
  src/state_estimator.cpp     # 주인 글로벌 위치 추정
  src/controller_base.cpp     # 모드 공통 부품 (상단yaw 락온/슬루/헤딩)
  src/idle_controller.cpp     # 모드0(IDLE) 정지 + 키보드 teleop
  src/follow_controller.cpp   # 모드1(FOLLOW) 선분 유지
  src/rotate_controller.cpp   # 모드2(ROTATE) 제자리 회전 추적
  src/control_node.cpp        # ROS 노드 (구독/발행/타이머)
  src/main.cpp
)
target_include_directories(control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(control_node
  rclcpp std_msgs geometry_msgs nav_msgs ros2_tracking_node
)

# 같은 패키지 안에서 생성한 메시지(ControlCmd/ProximityArray)를 노드에 링크.
#  Humble/Jazzy 공통으로 동작하는 방식: typesupport 타깃을 얻어 직접 링크.
#  (구버전의 rosidl_target_interfaces 보다 권장되는 현행 패턴)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(control_node "${cpp_typesupport_target}")

# -------- 설치 --------
install(TARGETS control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
# 키보드 teleop (파이썬, rclpy). ros2 run ros2_control_node teleop_keyboard 가능.
install(PROGRAMS scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_keyboard
)

ament_package()

cmake_minimum_required(VERSION 3.14)
project(ros2_tracking_node)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 커스텀 메시지 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OwnerPose.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs
)

# ByteTrack 라이브러리 경로
# (0525 수정: 절대경로 하드코딩 -> 링크 해제 후 상대 역산.
#  심볼릭 링크 워크스페이스(~/solcam_ws/src/ros2_tracking_node -> 실제 repo)에서는
#  "${CMAKE_CURRENT_SOURCE_DIR}/.." 를 바로 REALPATH하면 ".." 가 먼저 정규화되어
#  링크 위치(~/solcam_ws/src)로 잘못 풀린다. 따라서 먼저 self를 REALPATH로
#  링크 해제한 뒤(_PKG_REAL), 거기서 한 단계 위를 잡아야 원본 repo 루트가 나온다.)
get_filename_component(_PKG_REAL "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(BYTETRACK_DIR "${_PKG_REAL}/.." REALPATH)
set(BYTETRACK_LIB "${BYTETRACK_DIR}/build/libbytetrack.so")

add_executable(tracking_node src/tracking_node.cpp)

# 0511 C3 수정: install된 노드에서도 libbytetrack.so를 찾을 수 있게 RPATH 박음
set_target_properties(tracking_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${BYTETRACK_DIR}/build"
)

target_include_directories(tracking_node PRIVATE
  ${BYTETRACK_DIR}/include
)

target_link_libraries(tracking_node
  ${BYTETRACK_LIB}
)

ament_target_dependencies(tracking_node
  rclcpp
  std_msgs
  sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(tracking_node "${cpp_typesupport_target}")

install(TARGETS tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

# config 파일 설치
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

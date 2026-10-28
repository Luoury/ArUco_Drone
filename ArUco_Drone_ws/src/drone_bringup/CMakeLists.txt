cmake_minimum_required(VERSION 3.8)
project(drone_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Gazebo Harmonic -> ROS 2 桥接（替代 ABI 不匹配的 ros_gz_bridge）
add_executable(gz_sensor_bridge src/gz_sensor_bridge.cpp)
target_link_libraries(gz_sensor_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
ament_target_dependencies(gz_sensor_bridge
  rclcpp
  rosgraph_msgs
  sensor_msgs
  geometry_msgs
)

install(
  TARGETS gz_sensor_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

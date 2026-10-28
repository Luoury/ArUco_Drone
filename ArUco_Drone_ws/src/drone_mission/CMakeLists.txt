cmake_minimum_required(VERSION 3.8)
project(drone_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(drone_control REQUIRED)
find_package(drone_interfaces REQUIRED)

add_executable(mission_manager src/mission_manager.cpp)
ament_target_dependencies(mission_manager
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  mavros_msgs
  drone_interfaces
)

# drone_control 不能用 ament_target_dependencies：它把 hover_test 和库装在同一个
# export set 里，${drone_control_LIBRARIES} 展开出来是 drone_control::hover_test，
# CMake 会报 "links to target drone_control::hover_test but the target was not found"。
# 直接链导出的库目标，它自带 rclcpp/mavros_msgs 的 INTERFACE_LINK_LIBRARIES。
target_link_libraries(mission_manager drone_control::drone_control_lib)

install(TARGETS mission_manager DESTINATION lib/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(drone_nav CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(drone_control REQUIRED)

add_executable(waypoint_nav src/waypoint_nav.cpp)
target_link_libraries(waypoint_nav drone_control::drone_control_lib)
ament_target_dependencies(waypoint_nav rclcpp std_msgs std_srvs)

install(TARGETS waypoint_nav DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp std_msgs std_srvs)
ament_package()

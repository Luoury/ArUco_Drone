cmake_minimum_required(VERSION 3.10)
project(drone_aruco CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

set(ARUCO_BUILD_NODES ON)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/COLCON_IGNORE")
  set(ARUCO_BUILD_NODES OFF)
endif()

if(ARUCO_BUILD_NODES)
  find_package(rclcpp REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco)

  add_executable(aruco_detector src/aruco_detector.cpp)
  target_include_directories(aruco_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(aruco_detector rclcpp geometry_msgs sensor_msgs cv_bridge tf2)
  target_link_libraries(aruco_detector ${OpenCV_LIBS})

  install(TARGETS aruco_detector DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()

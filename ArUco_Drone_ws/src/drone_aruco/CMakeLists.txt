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

  # 降落状态机：只调 drone_control 库里的原语，不自己拼 mavros 报文
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(drone_control REQUIRED)

  add_executable(landing_controller src/landing_controller.cpp)
  # drone_control 不能放进 ament_target_dependencies：它的 export set 里同时有库和
  # hover_test 可执行文件，${drone_control_LIBRARIES} 会展开成 drone_control::hover_test，
  # CMake 报 "links to target drone_control::hover_test but the target was not found"。
  ament_target_dependencies(landing_controller rclcpp geometry_msgs std_msgs std_srvs)
  target_link_libraries(landing_controller drone_control::drone_control_lib)

  install(TARGETS aruco_detector landing_controller DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()

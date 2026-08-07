cmake_minimum_required(VERSION 3.8)
project(fp3_motion_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_demo_interfaces REQUIRED)

add_executable(fp3_motion_server_node src/fp3_motion_server_node.cpp)
target_compile_features(fp3_motion_server_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(fp3_motion_server_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  franka_demo_interfaces
)

install(TARGETS fp3_motion_server_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(fp3_moveit_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(franka_demo_interfaces REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(franka_msgs REQUIRED)

add_executable(scene_setup_node src/scene_setup_node.cpp)
target_compile_features(scene_setup_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(scene_setup_node
  rclcpp
  moveit_msgs
  shape_msgs
  geometry_msgs
)

add_executable(motion_server_node src/motion_server_node.cpp)
target_compile_features(motion_server_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(motion_server_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  franka_demo_interfaces
)

add_executable(command_router_node src/command_router_node.cpp)
target_compile_features(command_router_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(command_router_node
  rclcpp
  rclcpp_action
  franka_demo_interfaces
)

add_executable(pick_place_node src/pick_place_node.cpp)
target_compile_features(pick_place_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(pick_place_node
  rclcpp
  rclcpp_action
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  moveit_msgs
  shape_msgs
  geometry_msgs
  sensor_msgs
  franka_msgs
  franka_demo_interfaces
)

install(TARGETS
  scene_setup_node
  motion_server_node
  command_router_node
  pick_place_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(panda_motion_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(franka_demo_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(panda_motion_server_node src/panda_motion_server_node.cpp)
target_compile_features(panda_motion_server_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(panda_motion_server_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  geometry_msgs
  franka_demo_interfaces
  tf2_ros
  tf2_geometry_msgs
)

# Bridge to Isaac Sim (relays the planned trajectory on /joint_command),
# in place of a real ros2_control controller.
add_executable(joint_trajectory_bridge src/joint_trajectory_bridge.cpp)
target_compile_features(joint_trajectory_bridge PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(joint_trajectory_bridge
  rclcpp
  rclcpp_action
  control_msgs
  sensor_msgs
)

install(TARGETS panda_motion_server_node joint_trajectory_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

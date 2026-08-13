cmake_minimum_required(VERSION 3.8)
project(franka_demo_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecutePickTask.srv"
  "srv/GetFrames.srv"
  "srv/SegmentObject.srv"
  "srv/VisualizeSegmentation.srv"
  "srv/CreatePointcloud.srv"
  "srv/GenerateGraspPose.srv"
  "srv/VisualizeGrasps.srv"
  DEPENDENCIES sensor_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

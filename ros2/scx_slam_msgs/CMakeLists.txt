cmake_minimum_required(VERSION 3.16)
project(scx_slam_msgs VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StampedJob.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

cmake_minimum_required(VERSION 3.16)
project(scx_slam_workload VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scx_slam_executor REQUIRED)

add_executable(scx_slam_workload_smoke src/workload_smoke.cpp)
target_compile_features(scx_slam_workload_smoke PRIVATE cxx_std_17)
target_link_libraries(
  scx_slam_workload_smoke
  PRIVATE
    rclcpp::rclcpp
    scx_slam_executor::slamqos
)

install(TARGETS scx_slam_workload_smoke DESTINATION lib/${PROJECT_NAME})

ament_package()

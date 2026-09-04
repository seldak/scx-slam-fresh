cmake_minimum_required(VERSION 3.16)
project(scx_slam_workload VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scx_slam_executor REQUIRED)
find_package(scx_slam_msgs REQUIRED)

add_executable(scx_slam_workload_smoke src/workload_smoke.cpp)
target_compile_features(scx_slam_workload_smoke PRIVATE cxx_std_17)
target_link_libraries(
  scx_slam_workload_smoke
  PRIVATE
    rclcpp::rclcpp
    scx_slam_executor::freshness_executor
)

install(TARGETS scx_slam_workload_smoke DESTINATION lib/${PROJECT_NAME})

add_executable(scx_slam_pipeline src/slam_pipeline.cpp)
target_compile_features(scx_slam_pipeline PRIVATE cxx_std_17)
target_link_libraries(
  scx_slam_pipeline
  PRIVATE
    rclcpp::rclcpp
    scx_slam_executor::freshness_executor
    scx_slam_msgs::scx_slam_msgs
)

install(TARGETS scx_slam_pipeline DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(test_slam_pipeline
    COMMAND $<TARGET_FILE:scx_slam_pipeline> --duration 1
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 15
  )
  ament_add_test(test_slam_pipeline_window
    COMMAND $<TARGET_FILE:scx_slam_pipeline> --duration 1 --window-stats
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 15
  )
endif()

ament_package()

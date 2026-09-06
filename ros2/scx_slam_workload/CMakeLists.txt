# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(scx_slam_workload VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scx_slam_executor REQUIRED)
find_package(scx_slam_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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
target_include_directories(scx_slam_pipeline PRIVATE include)
target_compile_features(scx_slam_pipeline PRIVATE cxx_std_17)
target_link_libraries(
  scx_slam_pipeline
  PRIVATE
    rclcpp::rclcpp
    scx_slam_executor::freshness_executor
    scx_slam_msgs::scx_slam_msgs
)

install(TARGETS scx_slam_pipeline DESTINATION lib/${PROJECT_NAME})

add_executable(scx_slam_bag_adapter src/bag_adapter.cpp)
target_compile_features(scx_slam_bag_adapter PRIVATE cxx_std_17)
target_include_directories(
  scx_slam_bag_adapter PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(
  scx_slam_bag_adapter
  PRIVATE
    rclcpp::rclcpp
    scx_slam_msgs::scx_slam_msgs
    sensor_msgs::sensor_msgs
)

install(TARGETS scx_slam_bag_adapter DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_hint_ablation test/test_hint_ablation.cpp)
  target_include_directories(test_hint_ablation PRIVATE include)
  target_compile_features(test_hint_ablation PRIVATE cxx_std_17)
  target_link_libraries(test_hint_ablation scx_slam_executor::freshness_executor)
  ament_add_test(test_ros2_bag_accounting
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/test_ros2_bag_accounting.py
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 5
  )
  ament_add_gtest(test_bag_adapter test/test_bag_adapter.cpp)
  target_compile_features(test_bag_adapter PRIVATE cxx_std_17)
  target_include_directories(
    test_bag_adapter PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(
    test_bag_adapter
    builtin_interfaces::builtin_interfaces
    scx_slam_msgs::scx_slam_msgs
  )

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
  ament_add_test(test_ros2_eval_variants
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/../../scripts/run_ros2_eval.sh
      --check-variant-mappings
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 5
  )
endif()

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME})

ament_package()

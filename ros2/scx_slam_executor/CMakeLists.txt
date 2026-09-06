# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(scx_slam_executor VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_path(LIBBPF_INCLUDE_DIR bpf/bpf.h REQUIRED)
find_library(LIBBPF_LIBRARY bpf REQUIRED)

get_filename_component(SCX_SLAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(SCX_SLAM_BUILD_INCLUDE "${CMAKE_CURRENT_BINARY_DIR}/include/scx_slam_executor")

# Present the existing public API through the same namespaced include layout in
# both the build and install trees.  This keeps the ROS 2 package a thin wrapper
# around the standalone library instead of maintaining a second copy of it.
file(MAKE_DIRECTORY "${SCX_SLAM_BUILD_INCLUDE}")
configure_file(
  "${SCX_SLAM_ROOT}/src/slamqos.h"
  "${SCX_SLAM_BUILD_INCLUDE}/slamqos.h"
  COPYONLY
)
configure_file(
  "${SCX_SLAM_ROOT}/include/scx_slam_fresh_shared.h"
  "${SCX_SLAM_BUILD_INCLUDE}/scx_slam_fresh_shared.h"
  COPYONLY
)
configure_file(
  "${SCX_SLAM_ROOT}/include/scx_execution_trace.h"
  "${SCX_SLAM_BUILD_INCLUDE}/scx_execution_trace.h"
  COPYONLY
)

add_library(slamqos SHARED "${SCX_SLAM_ROOT}/src/slamqos.c")
add_library(scx_slam_executor::slamqos ALIAS slamqos)
target_compile_features(slamqos PUBLIC c_std_11)
target_include_directories(slamqos
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "${SCX_SLAM_ROOT}/src"
    "${SCX_SLAM_ROOT}/include"
    "${LIBBPF_INCLUDE_DIR}"
)
target_link_libraries(slamqos PUBLIC "${LIBBPF_LIBRARY}")
set_target_properties(slamqos PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 0)

add_library(freshness_executor SHARED src/freshness_executor.cpp)
add_library(scx_slam_executor::freshness_executor ALIAS freshness_executor)
target_compile_features(freshness_executor PUBLIC cxx_std_17)
target_include_directories(freshness_executor
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(freshness_executor PUBLIC slamqos rclcpp::rclcpp)
set_target_properties(
  freshness_executor PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 0
)

install(
  TARGETS slamqos freshness_executor
  EXPORT export_scx_slam_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(
  FILES
    "${SCX_SLAM_BUILD_INCLUDE}/slamqos.h"
    "${SCX_SLAM_BUILD_INCLUDE}/scx_slam_fresh_shared.h"
    "${SCX_SLAM_BUILD_INCLUDE}/scx_execution_trace.h"
  DESTINATION include/scx_slam_executor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)
  ament_add_gtest(test_slamqos test/test_slamqos.cpp)
  target_link_libraries(test_slamqos slamqos)

  ament_add_gtest(test_freshness_executor test/test_freshness_executor.cpp)
  target_link_libraries(test_freshness_executor freshness_executor std_msgs::std_msgs)
endif()

ament_export_targets(export_scx_slam_executor HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME})

ament_package()

# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.16)
project(scx_slam_executor VERSION 0.1.0 LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_path(LIBBPF_INCLUDE_DIR bpf/bpf.h REQUIRED)
find_library(LIBBPF_LIBRARY bpf REQUIRED)

get_filename_component(SCX_SLAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
if(DEFINED ENV{SCX_FRESH_DIR})
  set(SCX_FRESH_DEFAULT "$ENV{SCX_FRESH_DIR}")
else()
  set(SCX_FRESH_DEFAULT "${SCX_SLAM_ROOT}/../scx_fresh")
endif()
set(SCX_FRESH_DIR "${SCX_FRESH_DEFAULT}" CACHE PATH "External scx_fresh source checkout")
if(NOT EXISTS "${SCX_FRESH_DIR}/src/freshqos.c")
  message(FATAL_ERROR "Set SCX_FRESH_DIR to the scx_fresh checkout")
endif()
set(SCX_SLAM_BUILD_INCLUDE "${CMAKE_CURRENT_BINARY_DIR}/include/scx_slam_executor")

# Present the existing public API through the same namespaced include layout in
# both the build and install trees.  This keeps the ROS 2 package a thin wrapper
# around the standalone library instead of maintaining a second copy of it.
file(MAKE_DIRECTORY "${SCX_SLAM_BUILD_INCLUDE}")
configure_file(
  "${SCX_FRESH_DIR}/src/freshqos.h"
  "${SCX_SLAM_BUILD_INCLUDE}/freshqos.h"
  COPYONLY
)
configure_file(
  "${SCX_FRESH_DIR}/include/scx_fresh_shared.h"
  "${SCX_SLAM_BUILD_INCLUDE}/scx_fresh_shared.h"
  COPYONLY
)
configure_file(
  "${SCX_FRESH_DIR}/include/scx_execution_trace.h"
  "${SCX_SLAM_BUILD_INCLUDE}/scx_execution_trace.h"
  COPYONLY
)

add_library(freshqos SHARED "${SCX_FRESH_DIR}/src/freshqos.c")
add_library(scx_slam_executor::freshqos ALIAS freshqos)
target_compile_features(freshqos PUBLIC c_std_11)
target_include_directories(freshqos
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  PRIVATE
    "${SCX_FRESH_DIR}/src"
    "${SCX_FRESH_DIR}/include"
    "${LIBBPF_INCLUDE_DIR}"
)
target_link_libraries(freshqos PUBLIC "${LIBBPF_LIBRARY}")
set_target_properties(freshqos PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 0)

add_library(freshness_executor SHARED src/freshness_executor.cpp)
add_library(scx_slam_executor::freshness_executor ALIAS freshness_executor)
target_compile_features(freshness_executor PUBLIC cxx_std_17)
target_include_directories(freshness_executor
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(freshness_executor PUBLIC freshqos rclcpp::rclcpp)
set_target_properties(
  freshness_executor PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION 0
)

install(
  TARGETS freshqos freshness_executor
  EXPORT export_scx_slam_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(
  FILES
    "${SCX_SLAM_BUILD_INCLUDE}/freshqos.h"
    "${SCX_SLAM_BUILD_INCLUDE}/scx_fresh_shared.h"
    "${SCX_SLAM_BUILD_INCLUDE}/scx_execution_trace.h"
  DESTINATION include/scx_slam_executor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)
  ament_add_gtest(test_freshqos test/test_freshqos.cpp)
  target_link_libraries(test_freshqos freshqos)

  ament_add_gtest(test_freshness_executor test/test_freshness_executor.cpp)
  target_link_libraries(test_freshness_executor freshness_executor std_msgs::std_msgs)
endif()

ament_export_targets(export_scx_slam_executor HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../LICENSE"
  DESTINATION share/${PROJECT_NAME})

ament_package()

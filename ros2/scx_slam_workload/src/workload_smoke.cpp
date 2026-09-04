// SPDX-License-Identifier: GPL-2.0

#include <rclcpp/rclcpp.hpp>
#include <scx_slam_executor/slamqos.h>

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("scx_slam_workload_smoke");
  RCLCPP_INFO(
    node->get_logger(), "optional ROS 2 workload ready; pid_tgid=0x%016llx",
    static_cast<unsigned long long>(slamqos_pid_tgid_self()));
  rclcpp::shutdown();
  return 0;
}

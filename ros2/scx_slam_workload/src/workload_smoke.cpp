// SPDX-License-Identifier: MIT
#include <scx_slam_executor/application_stages.hpp>

#include <rclcpp/rclcpp.hpp>
#include <scx_slam_executor/freshness_executor.hpp>

#include <chrono>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>("scx_slam_workload_smoke");
  const auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  scx_slam_executor::FreshnessExecutor executor;

  scx_slam_executor::CallbackProfile profile;
  profile.stage_id = SLAM_STAGE_VISION_FE;
  profile.class_id = FRESH_CLASS_DEADLINE;
  profile.relative_deadline_ns = 33000000ULL;
  profile.stale_ns = 66000000ULL;
  profile.budget_ns = 12000000ULL;
  executor.add_callback_group_with_profile(group, node->get_node_base_interface(), profile);

  const auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1),
    [&]() {
      RCLCPP_INFO(node->get_logger(), "profiled callback executed on the handoff worker");
      executor.cancel();
    },
    group);

  executor.spin();
  RCLCPP_INFO(
    node->get_logger(), "optional ROS 2 workload ready; worker_pid_tgid=0x%016llx",
    static_cast<unsigned long long>(executor.worker_pid_tgid()));
  rclcpp::shutdown();
  return 0;
}

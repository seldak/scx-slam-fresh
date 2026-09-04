// SPDX-License-Identifier: GPL-2.0

#include <scx_slam_executor/freshness_executor.hpp>
#include <scx_slam_executor/slamqos.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace
{

class RecordingHintSink final : public scx_slam_executor::HintSink
{
public:
  void publish(uint64_t worker_pid_tgid, const slam_task_hint & hint) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    worker_ids_.push_back(worker_pid_tgid);
    hints_.push_back(hint);
    operations_.push_back(hint.job_id);
    published_.store(true);
  }

  void clear(uint64_t worker_pid_tgid) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cleared_worker_ids_.push_back(worker_pid_tgid);
    operations_.push_back(0);
  }

  bool published() const
  {
    return published_.load();
  }

  std::vector<uint64_t> worker_ids() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return worker_ids_;
  }

  std::vector<uint64_t> cleared_worker_ids() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cleared_worker_ids_;
  }

  std::vector<slam_task_hint> hints() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return hints_;
  }

  std::vector<uint64_t> operations() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return operations_;
  }

private:
  mutable std::mutex mutex_;
  std::atomic<bool> published_{false};
  std::vector<uint64_t> worker_ids_;
  std::vector<uint64_t> cleared_worker_ids_;
  std::vector<slam_task_hint> hints_;
  std::vector<uint64_t> operations_;
};

}  // namespace

TEST(FreshnessExecutor, PublishesAssignedJobBeforeCallbackAndClearsAfterward)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freshness_executor_test");
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sink = std::make_shared<RecordingHintSink>();
  scx_slam_executor::FreshnessExecutor executor(sink);

  scx_slam_executor::CallbackProfile profile;
  profile.stage_id = SLAM_STAGE_VISION_FE;
  profile.class_id = SLAM_SCX_CLASS_FE;
  profile.relative_deadline_ns = 33000000ULL;
  profile.stale_ns = 66000000ULL;
  profile.budget_ns = 12000000ULL;
  executor.add_callback_group_with_profile(group, node->get_node_base_interface(), profile);

  std::atomic<bool> callback_ran{false};
  std::atomic<uint64_t> callback_worker{0};
  const uint64_t dispatcher = slamqos_pid_tgid_self();
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1),
    [&]() {
      EXPECT_TRUE(sink->published());
      callback_worker.store(slamqos_pid_tgid_self());
      callback_ran.store(true);
      executor.cancel();
    },
    group);

  executor.spin();
  rclcpp::shutdown();

  ASSERT_TRUE(callback_ran.load());
  ASSERT_NE(executor.worker_pid_tgid(), 0U);
  EXPECT_EQ(callback_worker.load(), executor.worker_pid_tgid());
  EXPECT_NE(callback_worker.load(), dispatcher);
  const auto hints = sink->hints();
  const auto workers = sink->worker_ids();
  const auto cleared_workers = sink->cleared_worker_ids();
  ASSERT_EQ(hints.size(), 1U);
  ASSERT_EQ(workers.size(), 1U);
  ASSERT_EQ(cleared_workers.size(), 1U);
  EXPECT_EQ(workers.front(), executor.worker_pid_tgid());
  EXPECT_EQ(cleared_workers.front(), executor.worker_pid_tgid());
  EXPECT_EQ(hints.front().api_version, SLAM_SCX_API_VERSION);
  EXPECT_EQ(hints.front().stage_id, SLAM_STAGE_VISION_FE);
  EXPECT_EQ(hints.front().class_id, SLAM_SCX_CLASS_FE);
  EXPECT_EQ(hints.front().job_id, 1U);
  EXPECT_EQ(
    hints.front().deadline_ts_ns - hints.front().release_ts_ns,
    profile.relative_deadline_ns);
  EXPECT_EQ(hints.front().stale_ns, profile.stale_ns);
  EXPECT_EQ(hints.front().budget_ns, profile.budget_ns);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1, 0}));
}

TEST(FreshnessExecutor, ClearsAssignedJobWhenCallbackThrows)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freshness_executor_throw_test");
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sink = std::make_shared<RecordingHintSink>();
  scx_slam_executor::FreshnessExecutor executor(sink);
  executor.add_callback_group_with_profile(
    group, node->get_node_base_interface(), scx_slam_executor::CallbackProfile{});

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1),
    []() {throw std::runtime_error("expected callback failure");},
    group);

  EXPECT_THROW(executor.spin(), std::runtime_error);
  rclcpp::shutdown();

  EXPECT_EQ(sink->hints().size(), 1U);
  EXPECT_EQ(sink->cleared_worker_ids().size(), 1U);
}

TEST(FreshnessExecutor, ReplacesOnlyCompletedJobsWhileDraining)
{
  int argc = 0;
  char ** argv = nullptr;
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("freshness_executor_drain_test");
  auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sink = std::make_shared<RecordingHintSink>();
  scx_slam_executor::FreshnessExecutor executor(sink);
  executor.add_callback_group_with_profile(
    group, node->get_node_base_interface(), scx_slam_executor::CallbackProfile{});

  std::atomic<unsigned int> callback_count{0};
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(1),
    [&]() {
      if (++callback_count == 3) {
        executor.cancel();
      }
    },
    group);

  executor.spin();
  rclcpp::shutdown();

  EXPECT_EQ(callback_count.load(), 3U);
  EXPECT_EQ(sink->operations(), (std::vector<uint64_t>{1, 0, 2, 0, 3, 0}));
}

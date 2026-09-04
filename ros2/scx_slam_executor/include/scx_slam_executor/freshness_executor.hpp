// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <rclcpp/executor.hpp>
#include <scx_slam_executor/scx_slam_fresh_shared.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace scx_slam_executor
{

struct CallbackProfile
{
  uint32_t stage_id{SLAM_STAGE_MISC};
  uint32_t class_id{SLAM_SCX_CLASS_BE};
  uint64_t relative_deadline_ns{0};
  uint64_t stale_ns{0};
  uint64_t budget_ns{0};
  uint64_t slice_ns{0};
  uint32_t weight{0};
};

struct WorkerConfig
{
  // Negative values leave the inherited policy or affinity unchanged.
  int sched_ext_policy{-1};
  int cpu{-1};
};

struct MessageMetadata
{
  uint64_t job_id{0};
  uint64_t release_ts_ns{0};
};

using MessageMetadataExtractor =
  std::function<MessageMetadata(const std::shared_ptr<void> & message)>;

class HintSink
{
public:
  virtual ~HintSink() = default;

  virtual void publish(uint64_t worker_pid_tgid, const slam_task_hint & hint) = 0;
  virtual void clear(uint64_t worker_pid_tgid) = 0;
};

class NullHintSink final : public HintSink
{
public:
  void publish(uint64_t worker_pid_tgid, const slam_task_hint & hint) override;
  void clear(uint64_t worker_pid_tgid) override;
};

class PinnedMapHintSink final : public HintSink
{
public:
  explicit PinnedMapHintSink(const std::string & pin_dir);
  ~PinnedMapHintSink() override;

  PinnedMapHintSink(const PinnedMapHintSink &) = delete;
  PinnedMapHintSink & operator=(const PinnedMapHintSink &) = delete;

  void publish(uint64_t worker_pid_tgid, const slam_task_hint & hint) override;
  void clear(uint64_t worker_pid_tgid) override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// A deliberately single-worker executor implementing executor-contract v1.
// The spin thread selects ready ROS work; the worker only wakes after its hint
// has been published. A callback never migrates after execution starts.
class FreshnessExecutor final : public rclcpp::Executor
{
public:
  explicit FreshnessExecutor(
    std::shared_ptr<HintSink> hint_sink = std::make_shared<NullHintSink>(),
    WorkerConfig worker_config = {},
    const rclcpp::ExecutorOptions & options = rclcpp::ExecutorOptions());
  ~FreshnessExecutor() override;

  using rclcpp::Executor::add_callback_group;

  void add_callback_group_with_profile(
    const rclcpp::CallbackGroup::SharedPtr & group,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
    const CallbackProfile & profile,
    bool notify = true);

  void add_subscription_callback_group_with_erased_metadata(
    const rclcpp::CallbackGroup::SharedPtr & group,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
    const CallbackProfile & profile,
    MessageMetadataExtractor metadata_extractor,
    bool notify = true);

  template<typename MessageT, typename ExtractorT>
  void add_subscription_callback_group_with_profile(
    const rclcpp::CallbackGroup::SharedPtr & group,
    const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
    const CallbackProfile & profile,
    ExtractorT && metadata_extractor,
    bool notify = true)
  {
    add_subscription_callback_group_with_erased_metadata(
      group, node, profile,
      [extractor = std::forward<ExtractorT>(metadata_extractor)](
        const std::shared_ptr<void> & message) {
        return extractor(*std::static_pointer_cast<MessageT>(message));
      },
      notify);
  }

  void remove_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & group,
    bool notify = true) override;

  void spin() override;

  uint64_t worker_pid_tgid() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace scx_slam_executor

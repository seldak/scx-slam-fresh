// SPDX-License-Identifier: GPL-2.0

#include <scx_slam_executor/freshness_executor.hpp>
#include <scx_slam_executor/slamqos.h>

#include <linux/sched/types.h>
#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <unordered_map>
#include <utility>

namespace scx_slam_executor
{
namespace
{

uint64_t monotonic_now_ns()
{
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    throw std::system_error(errno, std::generic_category(), "clock_gettime(CLOCK_MONOTONIC)");
  }
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t add_saturating(uint64_t lhs, uint64_t rhs)
{
  if (rhs > std::numeric_limits<uint64_t>::max() - lhs) {
    return std::numeric_limits<uint64_t>::max();
  }
  return lhs + rhs;
}

void configure_worker(const WorkerConfig & config)
{
  if (config.cpu >= 0) {
    if (config.cpu >= CPU_SETSIZE) {
      throw std::invalid_argument("worker CPU is outside cpu_set_t");
    }
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(config.cpu, &cpuset);
    const int error = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (error != 0) {
      throw std::system_error(error, std::generic_category(), "pthread_setaffinity_np");
    }
  }

  if (config.sched_ext_policy >= 0) {
    sched_attr attr{};
    attr.size = sizeof(attr);
    attr.sched_policy = static_cast<uint32_t>(config.sched_ext_policy);
    if (syscall(SYS_sched_setattr, 0, &attr, 0) != 0) {
      throw std::system_error(errno, std::generic_category(), "sched_setattr(SCHED_EXT)");
    }
  }

  const int error = pthread_setname_np(pthread_self(), "scx_ros_worker");
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), "pthread_setname_np");
  }
}

slam_task_hint clear_hint()
{
  slam_task_hint hint{};
  hint.api_version = SLAM_SCX_API_VERSION;
  hint.stage_id = SLAM_STAGE_MISC;
  hint.class_id = SLAM_SCX_CLASS_BE;
  return hint;
}

}  // namespace

void NullHintSink::publish(uint64_t, const slam_task_hint &)
{
}

void NullHintSink::clear(uint64_t)
{
}

struct PinnedMapHintSink::Impl
{
  slamqos qos{-1};
};

PinnedMapHintSink::PinnedMapHintSink(const std::string & pin_dir)
: impl_(std::make_unique<Impl>())
{
  const int error = slamqos_open(&impl_->qos, pin_dir.c_str());
  if (error != 0) {
    throw std::system_error(-error, std::generic_category(), "slamqos_open");
  }
}

PinnedMapHintSink::~PinnedMapHintSink()
{
  slamqos_close(&impl_->qos);
}

void PinnedMapHintSink::publish(uint64_t worker_pid_tgid, const slam_task_hint & hint)
{
  const int error = slamqos_publish_hint_for(&impl_->qos, worker_pid_tgid, &hint);
  if (error != 0) {
    throw std::system_error(-error, std::generic_category(), "slamqos_publish_hint_for");
  }
}

void PinnedMapHintSink::clear(uint64_t worker_pid_tgid)
{
  publish(worker_pid_tgid, clear_hint());
}

struct FreshnessExecutor::Impl
{
  struct PendingWork
  {
    rclcpp::AnyExecutable executable;
    slam_task_hint hint{};
  };

  explicit Impl(std::shared_ptr<HintSink> sink, WorkerConfig config)
  : hint_sink(std::move(sink)), worker_config(config)
  {
  }

  std::shared_ptr<HintSink> hint_sink;
  WorkerConfig worker_config;
  mutable std::mutex mutex;
  std::condition_variable condition;
  std::optional<PendingWork> pending;
  bool worker_ready{false};
  bool worker_idle{false};
  bool stop{false};
  uint64_t worker_id{0};
  std::exception_ptr worker_error;

  std::mutex profiles_mutex;
  std::unordered_map<const rclcpp::CallbackGroup *, CallbackProfile> profiles;
  uint64_t next_job_id{1};

  CallbackProfile profile_for(const rclcpp::AnyExecutable & executable)
  {
    if (!executable.callback_group) {
      return {};
    }
    std::lock_guard<std::mutex> lock(profiles_mutex);
    const auto found = profiles.find(executable.callback_group.get());
    return found == profiles.end() ? CallbackProfile{} : found->second;
  }

  slam_task_hint make_hint(const CallbackProfile & profile)
  {
    const uint64_t release_ns = monotonic_now_ns();
    slam_task_hint hint{};
    hint.api_version = SLAM_SCX_API_VERSION;
    hint.stage_id = profile.stage_id;
    hint.class_id = profile.class_id;
    hint.job_id = next_job_id++;
    hint.release_ts_ns = release_ns;
    hint.deadline_ts_ns = profile.relative_deadline_ns == 0 ?
      0 : add_saturating(release_ns, profile.relative_deadline_ns);
    hint.stale_ns = profile.stale_ns;
    hint.budget_ns = profile.budget_ns;
    hint.slice_ns = profile.slice_ns;
    hint.weight = profile.weight;
    return hint;
  }
};

FreshnessExecutor::FreshnessExecutor(
  std::shared_ptr<HintSink> hint_sink,
  WorkerConfig worker_config,
  const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options),
  impl_(std::make_unique<Impl>(
      hint_sink ? std::move(hint_sink) : std::make_shared<NullHintSink>(), worker_config))
{
}

FreshnessExecutor::~FreshnessExecutor() = default;

void FreshnessExecutor::add_callback_group_with_profile(
  const rclcpp::CallbackGroup::SharedPtr & group,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
  const CallbackProfile & profile,
  bool notify)
{
  if (!group || !node) {
    throw std::invalid_argument("callback group and node must be non-null");
  }
  if (is_spinning()) {
    throw std::runtime_error("callback profiles cannot change while the executor is spinning");
  }
  if (profile.stage_id >= SLAM_STAGE_MAX) {
    throw std::invalid_argument("callback profile has an invalid stage_id");
  }
  if (profile.class_id != SLAM_SCX_CLASS_BE && profile.class_id != SLAM_SCX_CLASS_FE) {
    throw std::invalid_argument("callback profile has an invalid class_id");
  }

  rclcpp::Executor::add_callback_group(group, node, notify);
  std::lock_guard<std::mutex> lock(impl_->profiles_mutex);
  impl_->profiles[group.get()] = profile;
}

void FreshnessExecutor::remove_callback_group(
  const rclcpp::CallbackGroup::SharedPtr & group,
  bool notify)
{
  if (is_spinning()) {
    throw std::runtime_error("callback profiles cannot change while the executor is spinning");
  }
  rclcpp::Executor::remove_callback_group(group, notify);
  std::lock_guard<std::mutex> lock(impl_->profiles_mutex);
  impl_->profiles.erase(group.get());
}

void FreshnessExecutor::spin()
{
  bool expected = false;
  if (!spinning.compare_exchange_strong(expected, true)) {
    throw std::runtime_error("FreshnessExecutor::spin() called while already spinning");
  }

  {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->pending.reset();
    impl_->worker_ready = false;
    impl_->worker_idle = false;
    impl_->stop = false;
    impl_->worker_id = 0;
    impl_->worker_error = nullptr;
  }

  std::thread worker([this]() {
      try {
        configure_worker(impl_->worker_config);
        {
          std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->worker_id = slamqos_pid_tgid_self();
          impl_->worker_ready = true;
          impl_->worker_idle = true;
        }
        impl_->condition.notify_all();

        while (true) {
          std::optional<Impl::PendingWork> work;
          {
            std::unique_lock<std::mutex> lock(impl_->mutex);
            impl_->condition.wait(lock, [this]() {
              return impl_->stop || impl_->pending.has_value();
              });
            if (impl_->stop && !impl_->pending) {
              break;
            }
            work = std::move(impl_->pending);
            impl_->pending.reset();
          }

          try {
            execute_any_executable(work->executable);
          } catch (...) {
            // The callback is no longer in flight even when it throws. Do not
            // leave its identity in the worker's single-slot hint map.
            impl_->hint_sink->clear(impl_->worker_id);
            throw;
          }
          impl_->hint_sink->clear(impl_->worker_id);

          {
            std::lock_guard<std::mutex> lock(impl_->mutex);
            impl_->worker_idle = true;
          }
          impl_->condition.notify_all();
        }
      } catch (...) {
        {
          std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->worker_error = std::current_exception();
          impl_->worker_ready = true;
          impl_->worker_idle = true;
          impl_->stop = true;
        }
        impl_->condition.notify_all();
        try {
          cancel();
        } catch (...) {
        }
      }
    });

  std::exception_ptr dispatch_error;
  try {
    {
      std::unique_lock<std::mutex> lock(impl_->mutex);
      impl_->condition.wait(lock, [this]() {return impl_->worker_ready;});
      if (impl_->worker_error) {
        std::rethrow_exception(impl_->worker_error);
      }
    }

    while (rclcpp::ok(context_) && spinning.load()) {
      {
        std::unique_lock<std::mutex> lock(impl_->mutex);
        impl_->condition.wait(lock, [this]() {
            return impl_->worker_idle || impl_->worker_error || !spinning.load();
          });
        if (impl_->worker_error) {
          std::rethrow_exception(impl_->worker_error);
        }
        if (!spinning.load()) {
          break;
        }
      }

      rclcpp::AnyExecutable executable;
      if (!get_next_executable(executable)) {
        continue;
      }

      auto hint = impl_->make_hint(impl_->profile_for(executable));
      impl_->hint_sink->publish(impl_->worker_id, hint);

      {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->worker_idle = false;
        impl_->pending.emplace(Impl::PendingWork{std::move(executable), hint});
      }
      impl_->condition.notify_all();
    }
  } catch (...) {
    dispatch_error = std::current_exception();
  }

  spinning.store(false);
  {
    std::unique_lock<std::mutex> lock(impl_->mutex);
    impl_->condition.wait(lock, [this]() {return impl_->worker_idle || impl_->worker_error;});
    impl_->stop = true;
  }
  impl_->condition.notify_all();
  worker.join();

  if (dispatch_error) {
    std::rethrow_exception(dispatch_error);
  }
  if (impl_->worker_error) {
    std::rethrow_exception(impl_->worker_error);
  }
}

uint64_t FreshnessExecutor::worker_pid_tgid() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  return impl_->worker_id;
}

}  // namespace scx_slam_executor

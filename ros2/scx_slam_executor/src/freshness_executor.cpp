// SPDX-License-Identifier: MIT

#include <scx_slam_executor/freshness_executor.hpp>
#include <scx_slam_executor/freshqos.h>

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

fresh_task_hint clear_hint()
{
  fresh_task_hint hint{};
  hint.api_version = FRESH_API_VERSION;
  hint.stage_id = FRESH_STAGE_UNSPECIFIED;
  hint.class_id = FRESH_CLASS_BACKGROUND;
  return hint;
}

}  // namespace

void NullHintSink::publish(uint64_t, const fresh_task_hint &)
{
}

void NullHintSink::clear(uint64_t)
{
}

struct PinnedMapHintSink::Impl
{
  freshqos qos{-1};
};

PinnedMapHintSink::PinnedMapHintSink(const std::string & pin_dir)
: impl_(std::make_unique<Impl>())
{
  const int error = freshqos_open(&impl_->qos, pin_dir.c_str());
  if (error != 0) {
    throw std::system_error(-error, std::generic_category(), "freshqos_open");
  }
}

PinnedMapHintSink::~PinnedMapHintSink()
{
  freshqos_close(&impl_->qos);
}

void PinnedMapHintSink::publish(uint64_t worker_pid_tgid, const fresh_task_hint & hint)
{
  const int error = freshqos_publish_hint_for(&impl_->qos, worker_pid_tgid, &hint);
  if (error != 0) {
    throw std::system_error(-error, std::generic_category(), "freshqos_publish_hint_for");
  }
}

void PinnedMapHintSink::clear(uint64_t worker_pid_tgid)
{
  publish(worker_pid_tgid, clear_hint());
}

struct FreshnessExecutor::Impl
{
  struct ProfileBinding
  {
    CallbackProfile profile;
    MessageMetadataExtractor metadata_extractor;
    MessageObserver observer;
  };

  struct PendingWork
  {
    PendingWork() = default;
    PendingWork(const PendingWork &) = delete;
    PendingWork & operator=(const PendingWork &) = delete;
    PendingWork(PendingWork && other) noexcept
    : executable(other.executable), hint(other.hint), message(std::move(other.message)),
      message_info(std::move(other.message_info)), observer(std::move(other.observer)),
      reject_expired(other.reject_expired)
    {
      // AnyExecutable has a destructor but no move constructor. Its implicit
      // copy would let a moved-from destructor release our group's ownership.
      other.executable.callback_group.reset();
    }
    rclcpp::AnyExecutable executable;
    fresh_task_hint hint{};
    std::shared_ptr<void> message;
    std::unique_ptr<rclcpp::MessageInfo> message_info;
    MessageObserver observer;
    bool reject_expired{false};
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
  bool hint_needs_clear{false};
  bool stop{false};
  uint64_t worker_id{0};
  std::exception_ptr worker_error;

  std::mutex profiles_mutex;
  std::unordered_map<const rclcpp::CallbackGroup *, ProfileBinding> profiles;
  uint64_t next_job_id{1};

  // Shutdown only, after observing worker_idle under mutex. Between callbacks
  // replace the completed slot directly: condition.wait proves logical parking
  // but its mutex unlock can precede the actual futex sleep. An intermediate
  // BE clear could strand that runnable tail in a BE DSQ before the next hint.
  void clear_completed_hint()
  {
    if (hint_needs_clear) {
      hint_sink->clear(worker_id);
      hint_needs_clear = false;
    }
  }

  ProfileBinding binding_for(const rclcpp::AnyExecutable & executable)
  {
    if (!executable.callback_group) {
      return {};
    }
    std::lock_guard<std::mutex> lock(profiles_mutex);
    const auto found = profiles.find(executable.callback_group.get());
    return found == profiles.end() ? ProfileBinding{} : found->second;
  }

  fresh_task_hint make_hint(
    const CallbackProfile & profile,
    const std::optional<MessageMetadata> & metadata = std::nullopt)
  {
    const uint64_t release_ns = metadata ? metadata->release_ts_ns : monotonic_now_ns();
    fresh_task_hint hint{};
    hint.api_version = FRESH_API_VERSION;
    hint.stage_id = profile.stage_id;
    hint.class_id = profile.class_id;
    hint.job_id = metadata ? metadata->job_id : next_job_id++;
    hint.release_ts_ns = release_ns;
    hint.deadline_ts_ns = profile.relative_deadline_ns == 0 ?
      0 : add_saturating(release_ns, profile.relative_deadline_ns);
    hint.stale_ns = profile.stale_ns;
    hint.budget_ns = profile.budget_ns;
    hint.slice_ns = profile.slice_ns;
    hint.weight = profile.weight;
    return hint;
  }

  static bool expired(const PendingWork & work)
  {
    const auto & h = work.hint;
    if (!work.reject_expired) {
      return false;
    }
    const auto now = monotonic_now_ns();
    return (h.deadline_ts_ns && now > h.deadline_ts_ns) ||
           (h.stale_ns && now > h.release_ts_ns && now - h.release_ts_ns > h.stale_ns);
  }

  static void drop(PendingWork & work)
  {
    if (work.observer) {
      work.observer(work.message, MessageEvent::DroppedBeforeStart);
    }
    return_taken_message(work);
    release_callback_group(work.executable);
  }

  static void release_callback_group(rclcpp::AnyExecutable & executable)
  {
    if (executable.callback_group) {
      executable.callback_group->can_be_taken_from().store(true);
      executable.callback_group.reset();
    }
  }

  static void return_taken_message(PendingWork & work)
  {
    if (work.message && work.executable.subscription) {
      work.executable.subscription->return_message(work.message);
      work.message.reset();
    }
  }

  static void execute_taken_subscription(PendingWork & work)
  {
    try {
      work.executable.subscription->handle_message(work.message, *work.message_info);
    } catch (...) {
      return_taken_message(work);
      release_callback_group(work.executable);
      throw;
    }
    return_taken_message(work);
    release_callback_group(work.executable);
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
  if (profile.class_id != FRESH_CLASS_BACKGROUND && profile.class_id != FRESH_CLASS_DEADLINE &&
    profile.class_id != FRESH_CLASS_URGENT) {
    throw std::invalid_argument("callback profile has an invalid class_id");
  }

  rclcpp::Executor::add_callback_group(group, node, notify);
  std::lock_guard<std::mutex> lock(impl_->profiles_mutex);
  impl_->profiles[group.get()] = Impl::ProfileBinding{profile, {}};
}

void FreshnessExecutor::add_subscription_callback_group_with_erased_metadata(
  const rclcpp::CallbackGroup::SharedPtr & group,
  const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node,
  const CallbackProfile & profile,
  MessageMetadataExtractor metadata_extractor,
  bool notify,
  MessageObserver observer)
{
  if (!metadata_extractor) {
    throw std::invalid_argument("subscription metadata extractor must be callable");
  }
  add_callback_group_with_profile(group, node, profile, notify);
  std::lock_guard<std::mutex> lock(impl_->profiles_mutex);
  impl_->profiles[group.get()].metadata_extractor = std::move(metadata_extractor);
  impl_->profiles[group.get()].observer = std::move(observer);
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
    impl_->hint_needs_clear = false;
    impl_->stop = false;
    impl_->worker_id = 0;
    impl_->worker_error = nullptr;
  }

  std::thread worker([this]() {
      try {
        configure_worker(impl_->worker_config);
        {
          std::lock_guard<std::mutex> lock(impl_->mutex);
          impl_->worker_id = freshqos_pid_tgid_self();
          impl_->worker_ready = true;
        }
        impl_->condition.notify_all();

        while (true) {
          std::optional<Impl::PendingWork> work;
          {
            std::unique_lock<std::mutex> lock(impl_->mutex);
            impl_->worker_idle = true;
            impl_->condition.notify_all();
            impl_->condition.wait(lock, [this]() {
              return impl_->stop || impl_->pending.has_value();
              });
            if (impl_->stop && !impl_->pending) {
              break;
            }
            work.emplace(std::move(*impl_->pending));
            impl_->pending.reset();
          }

          try {
            if (work->message) {
              if (Impl::expired(*work)) {
                Impl::drop(*work);
              } else {
                Impl::execute_taken_subscription(*work);
              }
            } else {
              execute_any_executable(work->executable);
            }
          } catch (...) {
            Impl::return_taken_message(*work);
            Impl::release_callback_group(work->executable);
            // The callback is no longer in flight even when it throws. Do not
            // leave its identity in the worker's single-slot hint map.
            impl_->hint_sink->clear(impl_->worker_id);
            {
              std::lock_guard<std::mutex> lock(impl_->mutex);
              impl_->hint_needs_clear = false;
            }
            throw;
          }
          // Retain ownership through destruction of work and parking at the
          // top of the loop. Clearing here would expose a runnable BE tail.
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

      auto binding = impl_->binding_for(executable);
      Impl::PendingWork work;
      work.executable = std::move(executable);
      executable.callback_group.reset();

      try {
        if (binding.metadata_extractor) {
          if (!work.executable.subscription) {
            throw std::runtime_error(
                    "message metadata extractor registered for a non-subscription callback");
          }
          if (work.executable.subscription->get_delivered_message_kind() !=
            rclcpp::DeliveredMessageKind::ROS_MESSAGE)
          {
            throw std::runtime_error(
                    "message-aware handoff supports normal ROS messages only");
          }

          work.message = work.executable.subscription->create_message();
          work.message_info = std::make_unique<rclcpp::MessageInfo>();
          if (!work.executable.subscription->take_type_erased(
              work.message.get(), *work.message_info))
          {
            Impl::return_taken_message(work);
            Impl::release_callback_group(work.executable);
            continue;
          }

          const auto metadata = binding.metadata_extractor(work.message);
          if (metadata.job_id == 0 || metadata.release_ts_ns == 0) {
            throw std::runtime_error(
                    "message metadata must contain nonzero job_id and release_ts_ns");
          }
          work.hint = impl_->make_hint(binding.profile, metadata);
          work.reject_expired = binding.profile.reject_expired;
          work.observer = binding.observer;
          if (work.observer) {
            work.observer(work.message, MessageEvent::Selected);
          }
          if (Impl::expired(work)) {
            Impl::drop(work);
            continue;
          }
        } else {
          work.hint = impl_->make_hint(binding.profile);
        }

        impl_->hint_sink->publish(impl_->worker_id, work.hint);
      } catch (...) {
        Impl::return_taken_message(work);
        Impl::release_callback_group(work.executable);
        throw;
      }

      {
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->worker_idle = false;
        impl_->hint_needs_clear = true;
        impl_->pending.emplace(std::move(work));
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
    try {
      impl_->clear_completed_hint();
    } catch (...) {
      if (!dispatch_error) {dispatch_error = std::current_exception();}
    }
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

// SPDX-License-Identifier: MIT

#include <rclcpp/rclcpp.hpp>
#include <scx_slam_executor/freshness_executor.hpp>
#include <scx_slam_msgs/msg/stamped_job.hpp>
#include <scx_slam_workload/hint_ablation.hpp>
#include <scx_slam_workload/hog_metrics.hpp>

#include <linux/sched/types.h>
#include <pthread.h>
#include <sched.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using Message = scx_slam_msgs::msg::StampedJob;
using Executor = scx_slam_executor::FreshnessExecutor;

struct Options
{
  int duration_s{3};
  int warmup_s{0};
  int worker_cpu{-1};
  int ext_policy{-1};
  int hog_threads{0};
  bool window_stats{false};
  uint64_t source_start_ns{0};
  std::string pin_dir;
  std::string hint_mode{"auto"};
  std::string input_mode{"synthetic"};
};

struct StageStats
{
  std::mutex mutex;
  uint64_t offered{0};
  uint64_t arrivals{0};
  uint64_t executed{0};
  uint64_t dropped{0};
  uint64_t upstream_dropped{0};
  bool active{false};
  clockid_t active_clock{};
  uint64_t active_cpu_start{0};
  uint64_t completed{0};
  uint64_t late{0};
  uint64_t stale{0};
  uint64_t cpu_ns{0};
  uint64_t first_job_id{0};
  uint64_t last_job_id{0};
  uint64_t first_source_ts_ns{0};
  uint64_t last_source_ts_ns{0};
  std::vector<uint64_t> start_ages_ns;
  std::vector<uint64_t> completion_ages_ns;
};

struct StageSnapshot
{
  uint64_t offered{0};
  uint64_t arrivals{0};
  uint64_t executed{0};
  uint64_t dropped{0};
  uint64_t upstream_dropped{0};
  uint64_t completed{0};
  uint64_t late{0};
  uint64_t stale{0};
  uint64_t cpu_ns{0};
  uint64_t first_job_id{0};
  uint64_t last_job_id{0};
  uint64_t first_source_ts_ns{0};
  uint64_t last_source_ts_ns{0};
  uint64_t p99_start_age_ns{0};
  uint64_t max_start_age_ns{0};
  uint64_t p99_age_ns{0};
  uint64_t max_age_ns{0};
};

uint64_t clock_ns(clockid_t clock)
{
  timespec ts{};
  if (clock_gettime(clock, &ts) != 0) {
    throw std::system_error(errno, std::generic_category(), "clock_gettime");
  }
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t monotonic_ns()
{
  return clock_ns(CLOCK_MONOTONIC);
}

uint64_t thread_cpu_ns()
{
  return clock_ns(CLOCK_THREAD_CPUTIME_ID);
}

void busy_work_us(uint64_t work_us)
{
  const uint64_t start = thread_cpu_ns();
  const uint64_t duration = work_us * 1000ULL;
  while (thread_cpu_ns() - start < duration) {
    asm volatile("" ::: "memory");
  }
}

void sleep_until_ns(uint64_t wake_ns)
{
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(wake_ns / 1000000000ULL);
  ts.tv_nsec = static_cast<long>(wake_ns % 1000000000ULL);
  int error;
  do {
    error = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
  } while (error == EINTR);
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), "clock_nanosleep");
  }
}

uint64_t percentile_99(std::vector<uint64_t> values)
{
  if (values.empty()) {
    return 0;
  }
  std::sort(values.begin(), values.end());
  const size_t index = (values.size() * 99ULL + 99ULL) / 100ULL - 1ULL;
  return values[index];
}

void account_job(
  StageStats & stats, const Message & message, uint64_t work_us,
  uint64_t deadline_ns, uint64_t stale_ns, bool measured)
{
  const uint64_t start = monotonic_ns();
  const uint64_t start_age = start > message.release_ts_ns ? start - message.release_ts_ns : 0;
  clockid_t cpu_clock{};
  const int clock_error = pthread_getcpuclockid(pthread_self(), &cpu_clock);
  if (clock_error) {
    throw std::system_error(clock_error, std::generic_category(), "pthread_getcpuclockid");
  }
  const uint64_t cpu_start = thread_cpu_ns();
  if (measured) {
    std::lock_guard<std::mutex> lock(stats.mutex);
    stats.executed++;
    stats.active = true;
    stats.active_clock = cpu_clock;
    stats.active_cpu_start = cpu_start;
  }
  busy_work_us(work_us);
  const uint64_t cpu_ns = thread_cpu_ns() - cpu_start;

  const uint64_t now = monotonic_ns();
  if (!measured) {
    return;
  }
  const uint64_t age = now > message.release_ts_ns ? now - message.release_ts_ns : 0;
  std::lock_guard<std::mutex> lock(stats.mutex);
  stats.completed++;
  if (deadline_ns != 0 && age > deadline_ns) {
    stats.late++;
  }
  if (stale_ns != 0 && start_age > stale_ns) {
    stats.stale++;
  }
  stats.cpu_ns += cpu_ns;
  stats.active = false;
  stats.start_ages_ns.push_back(start_age);
  stats.completion_ages_ns.push_back(age);
}

bool source_time_measured(
  const Message & message, uint64_t source_start_ns,
  uint64_t warmup_ns, uint64_t duration_ns)
{
  const uint64_t start = source_start_ns + warmup_ns;
  return message.source_ts_ns >= start && message.source_ts_ns < start + duration_ns;
}

int parse_integer(const char * text, const char * option)
{
  char * end = nullptr;
  errno = 0;
  const long value = std::strtol(text, &end, 10);
  if (errno != 0 || !end || *end != '\0' || value < -1 || value > 86400) {
    throw std::invalid_argument(std::string("invalid value for ") + option);
  }
  return static_cast<int>(value);
}

uint64_t parse_u64(const char * text, const char * option)
{
  char * end = nullptr;
  errno = 0;
  const unsigned long long value = std::strtoull(text, &end, 10);
  if (errno != 0 || !end || *end != '\0' || text[0] == '-' || value == 0) {
    throw std::invalid_argument(std::string("invalid value for ") + option);
  }
  return static_cast<uint64_t>(value);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto require_value = [&](const char * option) {
        if (index + 1 >= argc) {
          throw std::invalid_argument(std::string("missing value for ") + option);
        }
        return argv[++index];
      };

    if (argument == "--duration") {
      options.duration_s = parse_integer(require_value("--duration"), "--duration");
      if (options.duration_s < 1) {
        throw std::invalid_argument("--duration must be at least one second");
      }
    } else if (argument == "--warmup") {
      options.warmup_s = parse_integer(require_value("--warmup"), "--warmup");
      if (options.warmup_s < 0) {
        throw std::invalid_argument("--warmup must be non-negative");
      }
    } else if (argument == "--worker-cpu") {
      options.worker_cpu = parse_integer(require_value("--worker-cpu"), "--worker-cpu");
    } else if (argument == "--ext-policy") {
      options.ext_policy = parse_integer(require_value("--ext-policy"), "--ext-policy");
    } else if (argument == "--hog") {
      options.hog_threads = parse_integer(require_value("--hog"), "--hog");
      if (options.hog_threads < 0) {
        throw std::invalid_argument("--hog must be non-negative");
      }
    } else if (argument == "--window-stats") {
      options.window_stats = true;
    } else if (argument == "--source-start-ns") {
      options.source_start_ns = parse_u64(
        require_value("--source-start-ns"), "--source-start-ns");
    } else if (argument == "--pin") {
      options.pin_dir = require_value("--pin");
    } else if (argument == "--hint-mode") {
      options.hint_mode = require_value("--hint-mode");
      if (options.hint_mode != "full" && options.hint_mode != "none" &&
        options.hint_mode != "imu-only" && options.hint_mode != "fe-only")
      {
        throw std::invalid_argument(
                "--hint-mode must be full, none, imu-only, or fe-only");
      }
    } else if (argument == "--input") {
      options.input_mode = require_value("--input");
      if (options.input_mode != "synthetic" && options.input_mode != "external") {
        throw std::invalid_argument("--input must be synthetic or external");
      }
    } else if (argument == "--help") {
      std::cout <<
        "usage: scx_slam_pipeline [--duration S] [--worker-cpu N] "
        "[--ext-policy N] [--pin DIR] [--hint-mode MODE] "
        "[--input synthetic|external] [--source-start-ns NS] [--warmup S] "
        "[--hog N] [--window-stats]\n";
      std::exit(0);
    } else if (argument != "--ros-args" && argument.rfind("__", 0) != 0) {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.input_mode == "external" && options.source_start_ns == 0) {
    throw std::invalid_argument("--input external requires --source-start-ns");
  }
  return options;
}

std::string effective_hint_mode(const Options & options)
{
  if (options.hint_mode != "auto") {
    return options.hint_mode;
  }
  return options.pin_dir.empty() ? "none" : "full";
}

scx_slam_executor::CallbackProfile profile(
  uint32_t stage, uint32_t class_id, uint64_t deadline_us,
  uint64_t stale_us, uint64_t budget_us)
{
  scx_slam_executor::CallbackProfile result;
  result.stage_id = stage;
  result.class_id = class_id;
  result.relative_deadline_ns = deadline_us * 1000ULL;
  result.stale_ns = stale_us * 1000ULL;
  result.budget_ns = budget_us * 1000ULL;
  return result;
}

scx_slam_executor::CallbackProfile stage_profile(
  const std::string & hint_mode, uint32_t stage, uint32_t class_id,
  uint64_t deadline_us, uint64_t stale_us, uint64_t budget_us)
{
  if (hint_mode == "imu-only" && stage != SLAM_STAGE_IMU_PREINT) {
    return profile(SLAM_STAGE_MISC, FRESH_CLASS_BACKGROUND, 0, 0, 0);
  }
  if (hint_mode == "fe-only" && stage == SLAM_STAGE_IMU_PREINT) {
    // Preserve IMU deadlines and budgets while removing Urgent service.
    return profile(SLAM_STAGE_MISC, FRESH_CLASS_DEADLINE, deadline_us, stale_us, budget_us);
  }
  return profile(stage, class_id, deadline_us, stale_us, budget_us);
}

scx_slam_executor::MessageMetadata metadata(const Message & message)
{
  return {message.job_id, message.release_ts_ns};
}

void publish_periodic(
  const rclcpp::Publisher<Message>::SharedPtr & publisher,
  uint64_t rate_hz, uint64_t duration_s, uint64_t start_ns)
{
  const uint64_t offered = rate_hz * duration_s;
  for (uint64_t index = 0; index < offered && rclcpp::ok(); ++index) {
    // Compute every release from the epoch. This preserves the exact offered
    // count for rates whose period is not an integer number of nanoseconds.
    const uint64_t release_ns = start_ns + index * 1000000000ULL / rate_hz;
    sleep_until_ns(release_ns);
    Message message;
    message.job_id = index + 1;
    message.release_ts_ns = release_ns;
    publisher->publish(message);
  }
}

StageSnapshot snapshot(StageStats & stats)
{
  std::lock_guard<std::mutex> lock(stats.mutex);
  StageSnapshot result;
  result.completed = stats.completed;
  result.offered = stats.offered;
  result.arrivals = stats.arrivals;
  result.executed = stats.executed;
  result.dropped = stats.dropped;
  result.upstream_dropped = stats.upstream_dropped;
  result.late = stats.late;
  result.stale = stats.stale;
  result.cpu_ns = stats.cpu_ns;
  if (stats.active) {
    result.cpu_ns += clock_ns(stats.active_clock) - stats.active_cpu_start;
  }
  result.first_job_id = stats.first_job_id;
  result.last_job_id = stats.last_job_id;
  result.first_source_ts_ns = stats.first_source_ts_ns;
  result.last_source_ts_ns = stats.last_source_ts_ns;
  result.p99_start_age_ns = percentile_99(stats.start_ages_ns);
  result.max_start_age_ns = stats.start_ages_ns.empty() ?
    0 : *std::max_element(stats.start_ages_ns.begin(), stats.start_ages_ns.end());
  result.p99_age_ns = percentile_99(stats.completion_ages_ns);
  result.max_age_ns = stats.completion_ages_ns.empty() ?
    0 : *std::max_element(stats.completion_ages_ns.begin(), stats.completion_ages_ns.end());
  return result;
}

void print_stats(
  const char * prefix, const char * name, uint64_t offered,
  const StageSnapshot & stats)
{
  const uint64_t resolved = stats.completed + stats.dropped + stats.upstream_dropped;
  const uint64_t unfinished = offered > resolved ? offered - resolved : 0;
  const uint64_t in_flight = stats.executed - stats.completed;
  std::cout << prefix << name << ": offered=" << offered <<
    " arrivals=" << stats.arrivals <<
    " executed=" << stats.executed <<
    " completed=" << stats.completed <<
    " dropped_before_start=" << stats.dropped <<
    " dropped_upstream=" << stats.upstream_dropped <<
    " late=" << stats.late <<
    " started_stale=" << stats.stale <<
    " unfinished=" << unfinished <<
    " in_flight=" << in_flight <<
    " pending=" << (unfinished > in_flight ? unfinished - in_flight : 0) <<
    " cpu_us=" << stats.cpu_ns / 1000ULL <<
    " first_job_id=" << stats.first_job_id <<
    " last_job_id=" << stats.last_job_id <<
    " first_source_ts_ns=" << stats.first_source_ts_ns <<
    " last_source_ts_ns=" << stats.last_source_ts_ns <<
    " p99_start_age_us=" << stats.p99_start_age_ns / 1000ULL <<
    " max_start_age_us=" << stats.max_start_age_ns / 1000ULL <<
    " p99_age_us=" << stats.p99_age_ns / 1000ULL <<
    " max_age_us=" << stats.max_age_ns / 1000ULL << '\n';
}

void configure_contender(int cpu, int ext_policy)
{
  if (cpu >= 0) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu, &cpuset);
    const int error = pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
    if (error != 0) {
      throw std::system_error(error, std::generic_category(), "pthread_setaffinity_np(hog)");
    }
  }
  if (ext_policy >= 0) {
    sched_attr attr{};
    attr.size = sizeof(attr);
    attr.sched_policy = static_cast<uint32_t>(ext_policy);
    if (syscall(SYS_sched_setattr, 0, &attr, 0) != 0) {
      throw std::system_error(errno, std::generic_category(), "sched_setattr(SCHED_EXT hog)");
    }
  }
  const int error = pthread_setname_np(pthread_self(), "scx_ros_hog");
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), "pthread_setname_np(hog)");
  }
}

void run_contender(
  std::atomic<bool> & running, int cpu, int ext_policy,
  std::vector<uint64_t> & completions,
  std::mutex & error_mutex, std::exception_ptr & background_error)
{
  try {
    configure_contender(cpu, ext_policy);
    while (running.load(std::memory_order_relaxed)) {
      busy_work_us(1000);
      completions.push_back(monotonic_ns());
    }
  } catch (...) {
    std::lock_guard<std::mutex> lock(error_mutex);
    if (!background_error) {
      background_error = std::current_exception();
    }
    running.store(false);
    rclcpp::shutdown();
  }
}

void wait_for_graph(
  const std::vector<rclcpp::Publisher<Message>::SharedPtr> & publishers)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    bool ready = true;
    for (const auto & publisher : publishers) {
      ready = ready && publisher->get_subscription_count() > 0;
    }
    if (ready) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("ROS graph did not connect the pipeline within two seconds");
}

void wait_for_external_sources(
  const rclcpp::Subscription<Message>::SharedPtr & imu_subscription,
  const rclcpp::Subscription<Message>::SharedPtr & camera_subscription)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    if (imu_subscription->get_publisher_count() > 0 &&
      camera_subscription->get_publisher_count() > 0)
    {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  throw std::runtime_error("external IMU and camera job publishers did not connect");
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
    const std::string hint_mode = effective_hint_mode(options);
    if (hint_mode != "none" && options.pin_dir.empty()) {
      throw std::invalid_argument("--hint-mode requires --pin unless mode is none");
    }
    rclcpp::init(argc, argv);

    std::shared_ptr<scx_slam_executor::HintSink> hint_sink;
    if (hint_mode == "none") {
      hint_sink = std::make_shared<scx_slam_executor::NullHintSink>();
    } else {
      hint_sink = std::make_shared<scx_slam_executor::PinnedMapHintSink>(options.pin_dir);
    }
    if (options.input_mode == "external") {
      hint_sink = std::make_shared<scx_slam_workload::AblationHintSink>(hint_sink, hint_mode);
    }
    const scx_slam_executor::WorkerConfig worker_config{
      options.ext_policy, options.worker_cpu};
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().durability_volatile();

    auto source_node = std::make_shared<rclcpp::Node>("slam_source");
    auto imu_node = std::make_shared<rclcpp::Node>("imu_propagation");
    auto vision_node = std::make_shared<rclcpp::Node>("vision_frontend");
    auto estimator_node = std::make_shared<rclcpp::Node>("state_estimator");
    auto mapping_node = std::make_shared<rclcpp::Node>("mapping_backend");

    auto imu_group = imu_node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto vision_group = vision_node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto estimator_group = estimator_node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto mapping_group = mapping_node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    rclcpp::Publisher<Message>::SharedPtr camera_source;
    rclcpp::Publisher<Message>::SharedPtr imu_source;
    if (options.input_mode == "synthetic") {
      camera_source = source_node->create_publisher<Message>("camera/jobs", qos);
      imu_source = source_node->create_publisher<Message>("imu/jobs", qos);
    }
    auto vision_output = vision_node->create_publisher<Message>("vision/jobs", qos);
    auto estimator_output = estimator_node->create_publisher<Message>("estimator/jobs", qos);

    StageStats imu_stats;
    StageStats vision_stats;
    StageStats estimator_stats;
    StageStats mapping_stats;
    std::atomic<uint64_t> measurement_start_ns{std::numeric_limits<uint64_t>::max()};
    std::atomic<uint64_t> measurement_end_ns{0};
    std::atomic<uint64_t> external_window_start_release_ns{0};
    const uint64_t warmup_ns = static_cast<uint64_t>(options.warmup_s) * 1000000000ULL;
    const uint64_t duration_ns = static_cast<uint64_t>(options.duration_s) * 1000000000ULL;

    auto measured_message = [&](const Message & message) {
        return options.input_mode == "external" ?
          source_time_measured(message, options.source_start_ns, warmup_ns, duration_ns) :
          message.release_ts_ns >= measurement_start_ns.load() &&
          message.release_ts_ns < measurement_end_ns.load();
      };
    // Independent source audit on the inherited housekeeping CPU. Every camera
    // offers one opportunity to each downstream stage, even if no worker runs.
    auto audit_node = std::make_shared<rclcpp::Node>("slam_source_audit");
    auto offer = [&](StageStats & stats, const Message & message) {
        std::lock_guard<std::mutex> lock(stats.mutex);
        stats.offered++;
        if (!stats.first_job_id) {
          stats.first_job_id = message.job_id;
          stats.first_source_ts_ns = message.source_ts_ns;
        }
        stats.last_job_id = message.job_id;
        stats.last_source_ts_ns = message.source_ts_ns;
      };
    auto audit_imu = audit_node->create_subscription<Message>("imu/jobs", qos,
      [&](const Message & message) {
        if (measured_message(message)) {offer(imu_stats, message);}
      });
    auto audit_camera = audit_node->create_subscription<Message>("camera/jobs", qos,
      [&](const Message & message) {
        if (measured_message(message)) {
          offer(vision_stats, message);
          offer(estimator_stats, message);
          offer(mapping_stats, message);
        }
      });
    rclcpp::executors::SingleThreadedExecutor audit_executor;
    audit_executor.add_node(audit_node);

    auto observe = [&](StageStats & stats, std::vector<StageStats *> downstream) {
        return [&, target = &stats, downstream](
          const std::shared_ptr<void> & erased, scx_slam_executor::MessageEvent event) {
            const auto & message = *std::static_pointer_cast<Message>(erased);
            if (!measured_message(message)) {return;}
            {
              std::lock_guard<std::mutex> lock(target->mutex);
              if (event == scx_slam_executor::MessageEvent::Selected) {
                target->arrivals++;
              } else {
                target->dropped++;
              }
            }
            if (event == scx_slam_executor::MessageEvent::DroppedBeforeStart) {
              // No fabricated downstream ROS message or callback: explicitly
              // resolve the source opportunity lost to this upstream eviction.
              for (auto * next : downstream) {
                std::lock_guard<std::mutex> lock(next->mutex);
                next->upstream_dropped++;
              }
            }
          };
      };
    auto admission_profile = [&](uint32_t stage, uint32_t class_id,
      uint64_t deadline_us, uint64_t stale_us, uint64_t budget_us) {
        auto result = options.input_mode == "external" ?
          profile(stage, class_id, deadline_us, stale_us, budget_us) :
          stage_profile(hint_mode, stage, class_id, deadline_us, stale_us, budget_us);
        result.reject_expired = options.input_mode == "external";
        return result;
      };

    rclcpp::SubscriptionOptions imu_subscription_options;
    imu_subscription_options.callback_group = imu_group;
    auto imu_subscription = imu_node->create_subscription<Message>(
      "imu/jobs", qos,
      [&](const Message::SharedPtr message) {
        const uint64_t window_start = measurement_start_ns.load(std::memory_order_acquire);
        const uint64_t window_end = measurement_end_ns.load(std::memory_order_acquire);
        const bool measured = options.input_mode == "external" ?
          source_time_measured(*message, options.source_start_ns, warmup_ns, duration_ns) :
          message->release_ts_ns >= window_start && message->release_ts_ns < window_end;
        if (measured) {
          uint64_t unset = 0;
          external_window_start_release_ns.compare_exchange_strong(
            unset, message->release_ts_ns, std::memory_order_acq_rel);
        }
        account_job(
          imu_stats, *message, 150, 5000000ULL, 10000000ULL, measured);
      },
      imu_subscription_options);

    rclcpp::SubscriptionOptions vision_subscription_options;
    vision_subscription_options.callback_group = vision_group;
    auto vision_subscription = vision_node->create_subscription<Message>(
      "camera/jobs", qos,
      [&](const Message::SharedPtr message) {
        const uint64_t window_start = measurement_start_ns.load(std::memory_order_acquire);
        const uint64_t window_end = measurement_end_ns.load(std::memory_order_acquire);
        const bool measured = options.input_mode == "external" ?
          source_time_measured(*message, options.source_start_ns, warmup_ns, duration_ns) :
          message->release_ts_ns >= window_start && message->release_ts_ns < window_end;
        const uint64_t work_us = 3000ULL + (message->job_id % 5ULL) * 500ULL;
        account_job(
          vision_stats, *message, work_us, 33000000ULL, 66000000ULL, measured);
        vision_output->publish(*message);
      },
      vision_subscription_options);

    rclcpp::SubscriptionOptions estimator_subscription_options;
    estimator_subscription_options.callback_group = estimator_group;
    auto estimator_subscription = estimator_node->create_subscription<Message>(
      "vision/jobs", qos,
      [&](const Message::SharedPtr message) {
        const uint64_t window_start = measurement_start_ns.load(std::memory_order_acquire);
        const uint64_t window_end = measurement_end_ns.load(std::memory_order_acquire);
        const bool measured = options.input_mode == "external" ?
          source_time_measured(*message, options.source_start_ns, warmup_ns, duration_ns) :
          message->release_ts_ns >= window_start && message->release_ts_ns < window_end;
        const uint64_t work_us = 2000ULL + (message->job_id % 5ULL) * 250ULL;
        account_job(
          estimator_stats, *message, work_us, 33000000ULL, 66000000ULL, measured);
        estimator_output->publish(*message);
      },
      estimator_subscription_options);

    rclcpp::SubscriptionOptions mapping_subscription_options;
    mapping_subscription_options.callback_group = mapping_group;
    auto mapping_subscription = mapping_node->create_subscription<Message>(
      "estimator/jobs", qos,
      [&](const Message::SharedPtr message) {
        const uint64_t window_start = measurement_start_ns.load(std::memory_order_acquire);
        const uint64_t window_end = measurement_end_ns.load(std::memory_order_acquire);
        const bool measured = options.input_mode == "external" ?
          source_time_measured(*message, options.source_start_ns, warmup_ns, duration_ns) :
          message->release_ts_ns >= window_start && message->release_ts_ns < window_end;
        account_job(mapping_stats, *message, 2000, 0, 0, measured);
      },
      mapping_subscription_options);

    std::vector<std::unique_ptr<Executor>> executors;
    executors.emplace_back(std::make_unique<Executor>(hint_sink, worker_config));
    executors.emplace_back(std::make_unique<Executor>(hint_sink, worker_config));
    executors.emplace_back(std::make_unique<Executor>(hint_sink, worker_config));
    executors.emplace_back(std::make_unique<Executor>(hint_sink, worker_config));

    executors[0]->add_subscription_callback_group_with_profile<Message>(
      imu_group, imu_node->get_node_base_interface(),
      admission_profile(SLAM_STAGE_IMU_PREINT, FRESH_CLASS_URGENT, 5000, 10000, 1000),
      metadata, true, observe(imu_stats, {}));
    executors[1]->add_subscription_callback_group_with_profile<Message>(
      vision_group, vision_node->get_node_base_interface(),
      admission_profile(SLAM_STAGE_VISION_FE, FRESH_CLASS_DEADLINE, 33000, 66000, 12000),
      metadata, true, observe(vision_stats, {&estimator_stats, &mapping_stats}));
    executors[2]->add_subscription_callback_group_with_profile<Message>(
      estimator_group, estimator_node->get_node_base_interface(),
      admission_profile(SLAM_STAGE_STATE_EST, FRESH_CLASS_DEADLINE, 33000, 66000, 10000),
      metadata, true, observe(estimator_stats, {&mapping_stats}));
    executors[3]->add_subscription_callback_group_with_profile<Message>(
      mapping_group, mapping_node->get_node_base_interface(),
      admission_profile(SLAM_STAGE_MAPPING_BE, FRESH_CLASS_BACKGROUND, 0, 0, 0),
      metadata, true, observe(mapping_stats, {}));

    std::mutex error_mutex;
    std::exception_ptr background_error;
    std::vector<std::thread> executor_threads;
    executor_threads.emplace_back([&]() {
        try {
          audit_executor.spin();
        } catch (...) {
          std::lock_guard<std::mutex> lock(error_mutex);
          if (!background_error) {background_error = std::current_exception();}
          rclcpp::shutdown();
        }
      });
    for (auto & executor : executors) {
      executor_threads.emplace_back([&executor, &error_mutex, &background_error]() {
          try {
            executor->spin();
          } catch (...) {
            std::lock_guard<std::mutex> lock(error_mutex);
            if (!background_error) {
              background_error = std::current_exception();
            }
            rclcpp::shutdown();
          }
        });
    }

    if (options.input_mode == "synthetic") {
      wait_for_graph({imu_source, camera_source, vision_output, estimator_output});
    } else {
      wait_for_graph({vision_output, estimator_output});
      wait_for_external_sources(imu_subscription, vision_subscription);
    }
    std::atomic<bool> contenders_running{true};
    std::vector<std::thread> contenders;
    std::vector<std::vector<uint64_t>> hog_completions(options.hog_threads);
    contenders.reserve(static_cast<size_t>(options.hog_threads));
    for (int index = 0; index < options.hog_threads; ++index) {
      hog_completions[index].reserve(
        static_cast<size_t>(options.duration_s + options.warmup_s + 30) * 1000);
      contenders.emplace_back(
        run_contender, std::ref(contenders_running), options.worker_cpu,
        options.ext_policy, std::ref(hog_completions[index]),
        std::ref(error_mutex), std::ref(background_error));
    }

    uint64_t start_ns = 0;
    uint64_t end_ns = 0;
    std::thread imu_generator;
    std::thread camera_generator;
    if (options.input_mode == "synthetic") {
      start_ns = monotonic_ns() + 100000000ULL;
      end_ns = start_ns + duration_ns;
      measurement_start_ns.store(start_ns, std::memory_order_release);
      measurement_end_ns.store(end_ns, std::memory_order_release);
      imu_generator = std::thread(
        publish_periodic, imu_source, 200ULL,
        static_cast<uint64_t>(options.duration_s), start_ns);
      camera_generator = std::thread(
        publish_periodic, camera_source, 30ULL,
        static_cast<uint64_t>(options.duration_s), start_ns);
      imu_generator.join();
      camera_generator.join();
    } else {
      const uint64_t start_timeout_ns =
        monotonic_ns() + warmup_ns + 10000000000ULL;
      while ((start_ns = external_window_start_release_ns.load(std::memory_order_acquire)) == 0) {
        if (monotonic_ns() >= start_timeout_ns) {
          contenders_running.store(false);
          for (auto & contender : contenders) {
            contender.join();
          }
          for (auto & executor : executors) {
            executor->cancel();
          }
          audit_executor.cancel();
          for (auto & thread : executor_threads) {
            thread.join();
          }
          rclcpp::shutdown();
          throw std::runtime_error("timed out waiting for external source-time window");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      end_ns = start_ns + duration_ns;
      measurement_start_ns.store(start_ns, std::memory_order_release);
      measurement_end_ns.store(end_ns, std::memory_order_release);
    }
    sleep_until_ns(end_ns);

    const auto window_imu = snapshot(imu_stats);
    const auto window_vision = snapshot(vision_stats);
    const auto window_estimator = snapshot(estimator_stats);
    const auto window_mapping = snapshot(mapping_stats);
    const uint64_t expected_imu = options.input_mode == "synthetic" ?
      static_cast<uint64_t>(options.duration_s) * 200ULL : window_imu.offered;
    const uint64_t expected_vision = options.input_mode == "synthetic" ?
      static_cast<uint64_t>(options.duration_s) * 30ULL : window_vision.offered;
    const uint64_t expected_estimator = expected_vision;
    const uint64_t expected_mapping = expected_vision;

    contenders_running.store(false);
    for (auto & contender : contenders) {
      contender.join();
    }

    if (!options.window_stats) {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    for (auto & executor : executors) {
      executor->cancel();
    }
    audit_executor.cancel();
    for (auto & thread : executor_threads) {
      thread.join();
    }
    rclcpp::shutdown();

    if (background_error) {
      std::rethrow_exception(background_error);
    }

    std::cout << "configuration: duration_s=" << options.duration_s <<
      " warmup_s=" << options.warmup_s <<
      " worker_cpu=" << options.worker_cpu <<
      " ext_policy=" << options.ext_policy <<
      " hog_threads=" << options.hog_threads <<
      " hint_mode=" << hint_mode <<
      " input=" << options.input_mode <<
      " source_start_ns=" << options.source_start_ns <<
      " window_stats=" << (options.window_stats ? 1 : 0) << '\n';

    if (options.input_mode == "external") {
      std::cout <<
        "external_counts: source-time window begins at source_start_ns plus warmup; "
        "offered counts independently observed source opportunities; arrivals counts "
        "subscription selections; dropped_upstream resolves upstream evictions\n";
    }

    if (options.window_stats) {
      for (size_t index = 0; index < hog_completions.size(); ++index) {
        std::cout << "hog_window: index=" << index <<
          " iterations=" << scx_slam_workload::window_iterations(
            hog_completions[index], start_ns, end_ns) <<
          " iteration_work_us=1000 start_ns=" << start_ns << " end_ns=" << end_ns << '\n';
      }
      print_stats("window_", "imu_prop", expected_imu, window_imu);
      print_stats("window_", "vision_fe", expected_vision, window_vision);
      print_stats("window_", "state_est", expected_estimator, window_estimator);
      print_stats("window_", "mapping_be", expected_mapping, window_mapping);
      return 0;
    }

    const auto final_imu = snapshot(imu_stats);
    const auto final_vision = snapshot(vision_stats);
    const auto final_estimator = snapshot(estimator_stats);
    const auto final_mapping = snapshot(mapping_stats);
    print_stats("", "imu_prop", expected_imu, final_imu);
    print_stats("", "vision_fe", expected_vision, final_vision);
    print_stats("", "state_est", expected_estimator, final_estimator);
    print_stats("", "mapping_be", expected_mapping, final_mapping);

    if (options.input_mode == "synthetic" &&
      (final_imu.completed != expected_imu ||
      final_vision.completed != expected_vision ||
      final_estimator.completed != expected_estimator ||
      final_mapping.completed != expected_mapping))
    {
      std::cerr << "pipeline did not drain every offered message\n";
      return 2;
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "scx_slam_pipeline: " << error.what() << '\n';
    return 1;
  }
}

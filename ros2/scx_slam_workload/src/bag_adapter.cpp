// SPDX-License-Identifier: MIT

#include <rclcpp/rclcpp.hpp>
#include <scx_slam_msgs/msg/stamped_job.hpp>
#include <scx_slam_workload/bag_adapter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <time.h>

#include <cerrno>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace
{

using Job = scx_slam_msgs::msg::StampedJob;

uint64_t clock_ns(clockid_t clock)
{
  timespec ts{};
  if (clock_gettime(clock, &ts) != 0) {
    throw std::system_error(errno, std::generic_category(), "clock_gettime");
  }
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

uint64_t monotonic_ns() {return clock_ns(CLOCK_MONOTONIC);}

class BagAdapter final : public rclcpp::Node
{
public:
  BagAdapter()
  : Node("scx_slam_bag_adapter")
  {
    trace_delivery_ = declare_parameter<bool>("trace_delivery", false);
    if (trace_delivery_) {delivery_trace_.reserve(trace_limit_);}
    const auto imu_input = declare_parameter<std::string>("imu_input", "/imu");
    const auto camera_input = declare_parameter<std::string>("camera_input", "/camera/image_raw");
    const auto imu_output = declare_parameter<std::string>("imu_output", "/imu/jobs");
    const auto camera_output = declare_parameter<std::string>("camera_output", "/camera/jobs");
    imu_index_ = scx_slam_workload::SourceJobIndex(
      declare_parameter<std::string>("imu_source_index", ""));
    camera_index_ = scx_slam_workload::SourceJobIndex(
      declare_parameter<std::string>("camera_source_index", ""));

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().durability_volatile();
    imu_jobs_ = create_publisher<Job>(imu_output, output_qos);
    camera_jobs_ = create_publisher<Job>(camera_output, output_qos);

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input, sensor_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message,
      const rclcpp::MessageInfo & info) {
        publish_job(imu_jobs_, imu_counts_, imu_index_, message->header.stamp, "IMU", info);
      });
    camera_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      camera_input, sensor_qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message,
      const rclcpp::MessageInfo & info) {
        publish_job(camera_jobs_, camera_counts_, camera_index_, message->header.stamp, "camera", info);
      });

    const auto required = declare_parameter<int>("required_job_subscribers", 0);
    if (required < 0) {throw std::invalid_argument("required_job_subscribers must be non-negative");}
    readiness_ = create_service<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("readiness_service", "~/ready"),
      [this, required](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
        const auto imu = imu_subscription_->get_publisher_count();
        const auto camera = camera_subscription_->get_publisher_count();
        const auto imu_jobs = imu_jobs_->get_subscription_count();
        const auto camera_jobs = camera_jobs_->get_subscription_count();
        response->success = imu == 1 && camera == 1 &&
          imu_jobs == static_cast<size_t>(required) && camera_jobs == static_cast<size_t>(required);
        response->message = "imu_publishers=" + std::to_string(imu) +
          " camera_publishers=" + std::to_string(camera) +
          " imu_job_subscribers=" + std::to_string(imu_jobs) +
          " camera_job_subscribers=" + std::to_string(camera_jobs);
      });

    RCLCPP_INFO(
      get_logger(), "bridging %s and %s to monotonic stamped jobs",
      imu_input.c_str(), camera_input.c_str());
  }

  void log_summary() const
  {
    log_stream_summary("imu", imu_counts_);
    log_stream_summary("camera", camera_counts_);
    // Flush after spinning stops: no per-message file I/O in the delivery path.
    if (trace_delivery_) {
      for (const auto & row : delivery_trace_) {
        std::cout << "delivery_trace: stream=" << row.stream << " job_id=" << row.job_id
                  << " source_ts_ns=" << row.source_ns
                  << " rmw_source_ns=" << row.rmw_source_ns
                  << " rmw_received_ns=" << row.rmw_received_ns
                  << " entry_mono_ns=" << row.entry_ns << " entry_real_ns=" << row.real_ns
                  << " release_mono_ns=" << row.release_ns
                  << " exit_mono_ns=" << row.exit_ns << " published=" << row.published << '\n';
      }
      std::cout << "delivery_trace_summary: records=" << delivery_trace_.size()
                << " omitted=" << trace_omitted_ << '\n';
    }
  }

private:
  struct DeliveryRecord
  {
    const char * stream{};
    uint64_t job_id{}, source_ns{}, entry_ns{}, real_ns{}, release_ns{}, exit_ns{};
    int64_t rmw_source_ns{}, rmw_received_ns{};
    bool published{};
  };
  static constexpr size_t trace_limit_ = 100000;
  bool trace_delivery_{false};
  size_t trace_omitted_{};
  std::vector<DeliveryRecord> delivery_trace_;

  struct StreamCounts
  {
    uint64_t received{0};
    uint64_t published{0};
    uint64_t dropped{0};
    uint64_t first_source_ts_ns{0};
  };

  void publish_job(
    const rclcpp::Publisher<Job>::SharedPtr & publisher, StreamCounts & counts,
    const scx_slam_workload::SourceJobIndex & index,
    const builtin_interfaces::msg::Time & source_stamp, const char * stream,
    const rclcpp::MessageInfo & info)
  {
    DeliveryRecord trace;
    if (trace_delivery_) {
      trace.entry_ns = monotonic_ns();
      trace.real_ns = clock_ns(CLOCK_REALTIME);
      trace.stream = stream;
      const auto & rmw = info.get_rmw_message_info();
      trace.rmw_source_ns = rmw.source_timestamp;
      trace.rmw_received_ns = rmw.received_timestamp;
    }
    uint64_t job_id = ++counts.received;
    try {
      job_id = index.job_id(scx_slam_workload::source_stamp_ns(source_stamp), job_id);
      const auto job = scx_slam_workload::make_stamped_job(job_id, monotonic_ns(), source_stamp);
      if (trace_delivery_) {
        trace.source_ns = job.source_ts_ns;
        trace.release_ns = job.release_ts_ns;
      }
      if (counts.first_source_ts_ns == 0) {
        counts.first_source_ts_ns = job.source_ts_ns;
      }
      publisher->publish(job);
      if (trace_delivery_) {trace.exit_ns = monotonic_ns(); trace.published = true;}
      counts.published++;
    } catch (const std::exception & error) {
      counts.dropped++;
      RCLCPP_ERROR(
        get_logger(), "dropping %s job %llu: %s", stream,
        static_cast<unsigned long long>(job_id), error.what());
    }
    if (trace_delivery_) {
      trace.job_id = job_id;
      if (!trace.exit_ns) {trace.exit_ns = monotonic_ns();}
      if (delivery_trace_.size() < trace_limit_) {delivery_trace_.push_back(trace);}
      else {++trace_omitted_;}
    }
  }

  void log_stream_summary(const char * stream, const StreamCounts & counts) const
  {
    RCLCPP_INFO(
      get_logger(),
      "adapter_%s: received=%llu published=%llu dropped=%llu first_source_ts_ns=%llu",
      stream, static_cast<unsigned long long>(counts.received),
      static_cast<unsigned long long>(counts.published),
      static_cast<unsigned long long>(counts.dropped),
      static_cast<unsigned long long>(counts.first_source_ts_ns));
  }

  StreamCounts imu_counts_;
  scx_slam_workload::SourceJobIndex imu_index_;
  scx_slam_workload::SourceJobIndex camera_index_;
  StreamCounts camera_counts_;
  rclcpp::Publisher<Job>::SharedPtr imu_jobs_;
  rclcpp::Publisher<Job>::SharedPtr camera_jobs_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr readiness_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto adapter = std::make_shared<BagAdapter>();
    rclcpp::spin(adapter);
    adapter->log_summary();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "scx_slam_bag_adapter: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}

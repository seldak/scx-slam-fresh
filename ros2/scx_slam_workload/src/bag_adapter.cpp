// SPDX-License-Identifier: GPL-2.0

#include <rclcpp/rclcpp.hpp>
#include <scx_slam_msgs/msg/stamped_job.hpp>
#include <scx_slam_workload/bag_adapter.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <time.h>

#include <cerrno>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>

namespace
{

using Job = scx_slam_msgs::msg::StampedJob;

uint64_t monotonic_ns()
{
  timespec ts{};
  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    throw std::system_error(errno, std::generic_category(), "clock_gettime(CLOCK_MONOTONIC)");
  }
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

class BagAdapter final : public rclcpp::Node
{
public:
  BagAdapter()
  : Node("scx_slam_bag_adapter")
  {
    const auto imu_input = declare_parameter<std::string>("imu_input", "/imu");
    const auto camera_input = declare_parameter<std::string>("camera_input", "/camera/image_raw");
    const auto imu_output = declare_parameter<std::string>("imu_output", "/imu/jobs");
    const auto camera_output = declare_parameter<std::string>("camera_output", "/camera/jobs");

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1000)).reliable().durability_volatile();
    imu_jobs_ = create_publisher<Job>(imu_output, output_qos);
    camera_jobs_ = create_publisher<Job>(camera_output, output_qos);

    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input, sensor_qos,
      [this](const sensor_msgs::msg::Imu::ConstSharedPtr message) {
        publish_job(imu_jobs_, imu_counts_, message->header.stamp, "IMU");
      });
    camera_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      camera_input, sensor_qos,
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
        publish_job(camera_jobs_, camera_counts_, message->header.stamp, "camera");
      });

    RCLCPP_INFO(
      get_logger(), "bridging %s and %s to monotonic stamped jobs",
      imu_input.c_str(), camera_input.c_str());
  }

  void log_summary() const
  {
    log_stream_summary("imu", imu_counts_);
    log_stream_summary("camera", camera_counts_);
  }

private:
  struct StreamCounts
  {
    uint64_t received{0};
    uint64_t published{0};
    uint64_t dropped{0};
  };

  void publish_job(
    const rclcpp::Publisher<Job>::SharedPtr & publisher, StreamCounts & counts,
    const builtin_interfaces::msg::Time & source_stamp, const char * stream)
  {
    const uint64_t job_id = ++counts.received;
    try {
      publisher->publish(
        scx_slam_workload::make_stamped_job(job_id, monotonic_ns(), source_stamp));
      counts.published++;
    } catch (const std::exception & error) {
      counts.dropped++;
      RCLCPP_ERROR(
        get_logger(), "dropping %s job %llu: %s", stream,
        static_cast<unsigned long long>(job_id), error.what());
    }
  }

  void log_stream_summary(const char * stream, const StreamCounts & counts) const
  {
    RCLCPP_INFO(
      get_logger(),
      "adapter_%s: received=%llu published=%llu dropped=%llu",
      stream, static_cast<unsigned long long>(counts.received),
      static_cast<unsigned long long>(counts.published),
      static_cast<unsigned long long>(counts.dropped));
  }

  StreamCounts imu_counts_;
  StreamCounts camera_counts_;
  rclcpp::Publisher<Job>::SharedPtr imu_jobs_;
  rclcpp::Publisher<Job>::SharedPtr camera_jobs_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
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

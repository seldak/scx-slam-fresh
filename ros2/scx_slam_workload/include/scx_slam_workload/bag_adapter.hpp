// SPDX-License-Identifier: GPL-2.0
#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <scx_slam_msgs/msg/stamped_job.hpp>

#include <cstdint>
#include <stdexcept>

namespace scx_slam_workload
{

inline uint64_t source_stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    throw std::invalid_argument("sensor timestamp is outside the ROS Time range");
  }
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

inline scx_slam_msgs::msg::StampedJob make_stamped_job(
  uint64_t job_id, uint64_t release_ts_ns,
  const builtin_interfaces::msg::Time & source_stamp)
{
  if (job_id == 0 || release_ts_ns == 0) {
    throw std::invalid_argument("job id and monotonic release timestamp must be nonzero");
  }
  scx_slam_msgs::msg::StampedJob job;
  job.job_id = job_id;
  job.release_ts_ns = release_ts_ns;
  job.source_ts_ns = source_stamp_ns(source_stamp);
  return job;
}

}  // namespace scx_slam_workload

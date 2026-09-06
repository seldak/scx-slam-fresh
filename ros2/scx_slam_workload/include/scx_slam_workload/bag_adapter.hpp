// SPDX-License-Identifier: MIT
#pragma once

#include <builtin_interfaces/msg/time.hpp>
#include <scx_slam_msgs/msg/stamped_job.hpp>

#include <cstdint>
#include <stdexcept>
#include <fstream>
#include <map>
#include <string>

namespace scx_slam_workload
{

// Exact bag ordinals preserve compute variation across playback sessions even
// when their first delivered messages differ. No timing/rate inference here.
class SourceJobIndex
{
public:
  explicit SourceJobIndex(const std::string & path = "")
  {
    if (path.empty()) {return;}
    std::ifstream input(path);
    if (!input) {throw std::runtime_error("cannot open source job index: " + path);}
    uint64_t stamp, ordinal, previous = 0;
    while (input >> stamp) {
      if (!(input >> ordinal) || !stamp || stamp <= previous ||
        ordinal != entries_.size() + 1)
      {
        throw std::runtime_error("invalid source job index: " + path);
      }
      entries_.emplace(stamp, ordinal);
      previous = stamp;
    }
    if (!input.eof() || entries_.empty()) {
      throw std::runtime_error("invalid source job index: " + path);
    }
  }

  uint64_t job_id(uint64_t source_ns, uint64_t receive_counter) const
  {
    if (entries_.empty()) {return receive_counter;}
    const auto found = entries_.find(source_ns);
    if (found == entries_.end()) {throw std::runtime_error("timestamp absent from bag index");}
    return found->second;
  }

private:
  std::map<uint64_t, uint64_t> entries_;
};

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

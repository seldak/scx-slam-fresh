// SPDX-License-Identifier: GPL-2.0

#include <gtest/gtest.h>
#include <scx_slam_workload/bag_adapter.hpp>

#include <stdexcept>

TEST(BagAdapter, PreservesSourceTimeAndUsesMonotonicRelease)
{
  builtin_interfaces::msg::Time source;
  source.sec = 123;
  source.nanosec = 456;

  const auto job = scx_slam_workload::make_stamped_job(7, 9001, source);
  EXPECT_EQ(job.job_id, 7U);
  EXPECT_EQ(job.release_ts_ns, 9001U);
  EXPECT_EQ(job.source_ts_ns, 123000000456ULL);
}

TEST(BagAdapter, AcceptsAZeroSourceStamp)
{
  builtin_interfaces::msg::Time source;
  const auto job = scx_slam_workload::make_stamped_job(1, 2, source);
  EXPECT_EQ(job.source_ts_ns, 0U);
}

TEST(BagAdapter, RejectsInvalidIdentityAndTime)
{
  builtin_interfaces::msg::Time source;
  EXPECT_THROW(scx_slam_workload::make_stamped_job(0, 1, source), std::invalid_argument);
  EXPECT_THROW(scx_slam_workload::make_stamped_job(1, 0, source), std::invalid_argument);

  source.sec = -1;
  EXPECT_THROW(scx_slam_workload::make_stamped_job(1, 1, source), std::invalid_argument);
}

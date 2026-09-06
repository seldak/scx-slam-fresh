// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <scx_slam_workload/hint_ablation.hpp>
#include <scx_slam_workload/hog_metrics.hpp>
#include <cstring>

TEST(HintAblation, CameraProjectionDoesNotChangeAdmissionHint)
{
  fresh_task_hint original{};
  original.stage_id = SLAM_STAGE_STATE_EST;
  original.class_id = FRESH_CLASS_DEADLINE;
  original.job_id = 61;
  original.release_ts_ns = 1000000000;
  original.deadline_ts_ns = 1033000000;
  original.stale_ns = 66000000;
  original.budget_ns = 10000000;
  original.flags = FRESH_HINT_EXECUTOR_OWNED;
  const auto saved = original;
  const auto a = scx_slam_workload::project_hint(original, "imu-only");
  EXPECT_EQ(std::memcmp(&saved, &original, sizeof(original)), 0);
  EXPECT_EQ(a.stage_id, SLAM_STAGE_MISC);
  EXPECT_EQ(a.class_id, FRESH_CLASS_BACKGROUND);
  EXPECT_EQ(a.deadline_ts_ns, 0U);
  EXPECT_EQ(a.stale_ns, 0U);
  EXPECT_EQ(a.budget_ns, 0U);
  EXPECT_EQ(a.job_id, original.job_id);
  EXPECT_EQ(a.release_ts_ns, original.release_ts_ns);
  EXPECT_EQ(a.flags, original.flags);
  for (const auto & mode : {"full", "fe-only"}) {
    const auto h = scx_slam_workload::project_hint(original, mode);
    EXPECT_EQ(std::memcmp(&saved, &h, sizeof(h)), 0);
  }
}

TEST(HintAblation, ImuAdmissionAndAgeProtectionSurviveRemovalOfDedicatedLane)
{
  fresh_task_hint imu{};
  imu.stage_id = SLAM_STAGE_IMU_PREINT;
  imu.class_id = FRESH_CLASS_URGENT;
  imu.deadline_ts_ns = 1005000000;
  imu.stale_ns = 10000000;
  imu.budget_ns = 1000000;
  const auto a = scx_slam_workload::project_hint(imu, "imu-only");
  EXPECT_EQ(std::memcmp(&a, &imu, sizeof(imu)), 0);
  const auto b = scx_slam_workload::project_hint(imu, "fe-only");
  EXPECT_EQ(b.stage_id, SLAM_STAGE_MISC);
  EXPECT_EQ(b.class_id, FRESH_CLASS_DEADLINE);
  EXPECT_EQ(b.deadline_ts_ns, imu.deadline_ts_ns);
  EXPECT_EQ(b.stale_ns, imu.stale_ns);
  EXPECT_EQ(b.budget_ns, imu.budget_ns);
  EXPECT_EQ(b.flags, FRESH_HINT_EXECUTOR_OWNED);
  EXPECT_EQ(imu.flags, 0U);
}

TEST(HogMetrics, UsesRecordedCompletionEventsInHalfOpenWallWindow)
{
  const std::vector<uint64_t> stamps{9, 10, 12, 19, 20, 21};
  EXPECT_EQ(scx_slam_workload::window_iterations(stamps, 10, 20), 3U);
  EXPECT_EQ(scx_slam_workload::window_iterations(stamps, 10, 10), 0U);
  EXPECT_EQ(scx_slam_workload::window_iterations({}, 10, 20), 0U);
}

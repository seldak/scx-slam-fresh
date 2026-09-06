// SPDX-License-Identifier: MIT
#pragma once

#include <scx_slam_executor/application_stages.hpp>
#include <scx_slam_executor/freshness_executor.hpp>
#include <string>

namespace scx_slam_workload
{
// Transport projection only: the executor still selects/drops against the
// original application profile. Never mutate that profile or the input hint.
inline fresh_task_hint project_hint(const fresh_task_hint & original, const std::string & mode)
{
  auto hint = original;
  if (mode == "imu-only" && hint.stage_id != SLAM_STAGE_IMU_PREINT) {
    hint.stage_id = SLAM_STAGE_MISC;
    hint.class_id = FRESH_CLASS_BACKGROUND;
    hint.deadline_ts_ns = 0;
    hint.stale_ns = 0;
    hint.budget_ns = 0;
    hint.slice_ns = 0;
    hint.weight = 0;
  } else if (mode == "fe-only" && hint.stage_id == SLAM_STAGE_IMU_PREINT) {
    hint.stage_id = SLAM_STAGE_MISC;
    hint.class_id = FRESH_CLASS_DEADLINE;
    // Preserve the real IMU's age-demotion exemption without its DSQ/preempt
    // route. Otherwise this ablation would reintroduce the ownership lock.
    hint.flags |= FRESH_HINT_EXECUTOR_OWNED;
  }
  return hint;
}

class AblationHintSink final : public scx_slam_executor::HintSink
{
public:
  AblationHintSink(std::shared_ptr<scx_slam_executor::HintSink> sink, std::string mode)
  : sink_(std::move(sink)), mode_(std::move(mode)) {}
  void publish(uint64_t worker, const fresh_task_hint & hint) override
  {
    sink_->publish(worker, project_hint(hint, mode_));
  }
  void clear(uint64_t worker) override {sink_->clear(worker);}
private:
  std::shared_ptr<scx_slam_executor::HintSink> sink_;
  std::string mode_;
};
}

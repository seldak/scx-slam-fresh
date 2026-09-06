// SPDX-License-Identifier: MIT
#pragma once
#include <stdint.h>

// Synthetic workload identities; never scheduler service classes.
enum slam_stage_id : uint32_t {
    SLAM_STAGE_IMU_PREINT     = 0,
    SLAM_STAGE_VISION_FE      = 1,
    SLAM_STAGE_STATE_EST      = 2,
    SLAM_STAGE_MAPPING_BE     = 3,
    SLAM_STAGE_LOOP_CLOSURE   = 4,
    SLAM_STAGE_LIDAR_PREINT  = 5,
    SLAM_STAGE_LIDAR_REG       = 6,
    SLAM_STAGE_MISC           = 15,
    SLAM_STAGE_MAX            = 16
};

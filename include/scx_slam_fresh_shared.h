/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared types between BPF and userspace.
 *
 * Keep this file small and stable: it's part of the "API surface" of the project.
 */

#pragma once

#ifdef __VMLINUX_H__
/* BPF side: vmlinux.h is included first, so use kernel types and avoid libc typedefs. */
typedef __u8  uint8_t;
typedef __u16 uint16_t;
typedef __u32 uint32_t;
typedef __u64 uint64_t;
#else
/* Userspace side */
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Versioning: bump when struct layouts change in an incompatible way. */
#define SLAM_SCX_API_VERSION 1

/* Scheduling class hint (userspace -> BPF). */
enum slam_scx_class : uint32_t {
    SLAM_SCX_CLASS_BE    = 0,  /* best-effort / background */
    SLAM_SCX_CLASS_FE    = 1,  /* front-end / latency & freshness sensitive */
};

/* Stages are intentionally generic: adapt to your pipeline. */
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

struct slam_task_hint {
    uint32_t api_version;      /* must be SLAM_SCX_API_VERSION */
    uint32_t stage_id;         /* enum slam_stage_id */
    uint32_t class_id;         /* enum slam_scx_class */
    uint32_t flags;            /* reserved */

    uint64_t job_id;           /* monotonic per stage/thread */
    uint64_t release_ts_ns;    /* when job became ready (CLOCK_MONOTONIC) */
    uint64_t deadline_ts_ns;   /* absolute deadline; 0 => none */
    uint64_t stale_ns;         /* freshness window; 0 => never stale */

    uint64_t budget_ns;        /* 0 => no budget enforcement */
    uint64_t slice_ns;         /* 0 => scheduler default */

    uint32_t weight;           /* BE fairness weight; 0 => default */
    uint32_t _pad;
};

/* Events (BPF -> userspace). */
enum slam_evt_kind : uint32_t {
    SLAM_EVT_DEADLINE_MISS   = 1,
    SLAM_EVT_BUDGET_OVERRUN  = 2,
    SLAM_EVT_STALE_DEMOTION  = 3,
    SLAM_EVT_BUDGET_DEMOTION = 4,
};

struct slam_evt {
    uint64_t ts_ns;
    uint64_t pid_tgid;

    uint32_t stage_id;
    uint32_t kind;

    uint64_t job_id;

    uint64_t release_ts_ns;
    uint64_t deadline_ts_ns;

    uint64_t exec_ns_in_job;
    uint64_t age_ns;

    uint32_t class_id;
    uint32_t _pad;
};

#ifdef __cplusplus
}
#endif

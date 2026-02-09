/* SPDX-License-Identifier: GPL-2.0 */
#pragma once

#include <stdint.h>
#include "../include/scx_slam_fresh_shared.h"

#ifdef __cplusplus
extern "C" {
#endif

struct slamqos {
    int map_fd;
};

/* PID/TID packed key used by the BPF map (tgid<<32 | tid). */
uint64_t slamqos_pid_tgid_self(void);

/* Open pinned maps under pin_dir (expects task_hints pinned at pin_dir/task_hints). */
int slamqos_open(struct slamqos *q, const char *pin_dir);

/* Close map fds. */
void slamqos_close(struct slamqos *q);

/* Publish/overwrite the hint for the current thread (pid/tgid key). */
int slamqos_publish_hint(struct slamqos *q, const struct slam_task_hint *hint);

/* Publish/overwrite the hint for an explicit pid/tgid key (used for producer-driven hinting). */
int slamqos_publish_hint_for(struct slamqos *q, uint64_t pid_tgid, const struct slam_task_hint *hint);

/* Clear current thread's hint (sets to BE/MISC with job_id=0). */
int slamqos_clear_hint(struct slamqos *q);

/* Convenience helper: build and publish a hint. */
int slamqos_publish_job(struct slamqos *q,
                        uint32_t stage_id,
                        uint32_t class_id,
                        uint64_t job_id,
                        uint64_t release_ts_ns,
                        uint64_t deadline_ts_ns,
                        uint64_t stale_ns,
                        uint64_t budget_ns,
                        uint64_t slice_ns,
                        uint32_t weight);

/* Convenience helper: build+publish a hint for a specific pid_tgid. */
int slamqos_publish_job_for(struct slamqos *q,
                           uint64_t pid_tgid,
                           uint32_t stage_id,
                           uint32_t class_id,
                           uint64_t job_id,
                           uint64_t release_ts_ns,
                           uint64_t deadline_ts_ns,
                           uint64_t stale_ns,
                           uint64_t budget_ns,
                           uint64_t slice_ns,
                           uint32_t weight);

#ifdef __cplusplus
}
#endif

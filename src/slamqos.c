/* SPDX-License-Identifier: GPL-2.0 */
#include "slamqos.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <linux/limits.h>

#include <bpf/bpf.h>

uint64_t slamqos_pid_tgid_self(void)
{
    uint32_t tgid = (uint32_t)getpid();
    uint32_t tid  = (uint32_t)syscall(SYS_gettid);
    return ((uint64_t)tgid << 32) | tid;
}


int slamqos_open(struct slamqos *q, const char *pin_dir)
{
    if (!q || !pin_dir)
        return -EINVAL;

    char path[PATH_MAX];
    snprintf(path, sizeof(path), "%s/task_hints", pin_dir);

    q->map_fd = bpf_obj_get(path);
    if (q->map_fd < 0) {
        fprintf(stderr, "slamqos_open: bpf_obj_get(%s) failed: %s\n",
                path, strerror(errno));
        return -errno;
    }
    return 0;
}

void slamqos_close(struct slamqos *q)
{
    if (!q)
        return;
    if (q->map_fd >= 0)
        close(q->map_fd);
    q->map_fd = -1;
}

int slamqos_publish_hint(struct slamqos *q, const struct slam_task_hint *hint)
{
    if (!q || q->map_fd < 0 || !hint)
        return -EINVAL;

    uint64_t key = slamqos_pid_tgid_self();
    int err = bpf_map_update_elem(q->map_fd, &key, hint, BPF_ANY);
    if (err) {
        fprintf(stderr, "slamqos_publish_hint: update failed: %s\n", strerror(errno));
        return -errno;
    }
    return 0;
}

int slamqos_publish_hint_for(struct slamqos *q, uint64_t pid_tgid, const struct slam_task_hint *hint)
{
    if (!q || q->map_fd < 0 || !hint || !pid_tgid)
        return -EINVAL;

    uint64_t key = pid_tgid;
    int err = bpf_map_update_elem(q->map_fd, &key, hint, BPF_ANY);
    if (err) {
        fprintf(stderr, "slamqos_publish_hint_for: update failed: %s\n", strerror(errno));
        return -errno;
    }
    return 0;
}

int slamqos_publish_job(struct slamqos *q,
                        uint32_t stage_id,
                        uint32_t class_id,
                        uint64_t job_id,
                        uint64_t release_ts_ns,
                        uint64_t deadline_ts_ns,
                        uint64_t stale_ns,
                        uint64_t budget_ns,
                        uint64_t slice_ns,
                        uint32_t weight)
{
    struct slam_task_hint h = {};
    h.api_version = SLAM_SCX_API_VERSION;
    h.stage_id = stage_id;
    h.class_id = class_id;
    h.job_id = job_id;
    h.release_ts_ns = release_ts_ns;
    h.deadline_ts_ns = deadline_ts_ns;
    h.stale_ns = stale_ns;
    h.budget_ns = budget_ns;
    h.slice_ns = slice_ns;
    h.weight = weight;
    return slamqos_publish_hint(q, &h);
}
int slamqos_clear_hint(struct slamqos *q)
{
    struct slam_task_hint h = {};
    h.api_version = SLAM_SCX_API_VERSION;
    h.stage_id = SLAM_STAGE_MISC;
    h.class_id = SLAM_SCX_CLASS_BE;
    h.job_id = 0;
    h.release_ts_ns = 0;
    h.deadline_ts_ns = 0;
    h.stale_ns = 0;
    h.budget_ns = 0;
    h.slice_ns = 0;
    h.weight = 0;
    return slamqos_publish_hint(q, &h);
}
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
                           uint32_t weight)
{
    struct slam_task_hint h = {};
    h.api_version = SLAM_SCX_API_VERSION;
    h.stage_id = stage_id;
    h.class_id = class_id;
    h.job_id = job_id;
    h.release_ts_ns = release_ts_ns;
    h.deadline_ts_ns = deadline_ts_ns;
    h.stale_ns = stale_ns;
    h.budget_ns = budget_ns;
    h.slice_ns = slice_ns;
    h.weight = weight;
    return slamqos_publish_hint_for(q, pid_tgid, &h);
}

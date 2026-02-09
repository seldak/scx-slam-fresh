/* SPDX-License-Identifier: GPL-2.0 */
/*
 * scx_slam_fresh: freshness-aware sched_ext scheduler for SLAM-ish pipelines.
 *
 * Policy summary:
 * - Front-end (FE) tasks: EDF-like ordering using DSQ vtime = effective deadline.
 * - Stale work: demoted to DSQ_STALE.
 * - Budget overrun: FE task is demoted to BE for the remainder of the job.
 * - Back-end (BE): vtime fairness using vruntime.
 *
 * This is deliberately a "readable" scheduler, intended for learning + portfolio.
 */

#include "vmlinux.h"

#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include <bpf/bpf_core_read.h>

#include "scx_slam_fresh_shared.h"

char LICENSE[] SEC("license") = "GPL";

/* -----------------------------
 * Minimal helper / compat layer
 * -----------------------------
 *
 * sched_ext kfuncs were renamed across kernel versions (aliases may later vanish).
 * We use weak ksyms + bpf_ksym_exists() to call whichever name exists.
 *
 * Renames include:
 * - scx_bpf_dispatch() -> scx_bpf_dsq_insert() (alias removed in newer kernels)
 * - scx_bpf_dispatch_vtime() -> scx_bpf_dsq_insert_vtime()
 * - scx_bpf_consume() -> scx_bpf_dsq_move_to_local()
 */

#ifndef __weak
#define __weak __attribute__((weak))
#endif

#ifndef bpf_ksym_exists
#define bpf_ksym_exists(sym) ({ \
    _Static_assert(!__builtin_constant_p(!!sym), #sym " should be marked as __weak"); \
    !!sym; \
})
#endif

#ifndef BPF_STRUCT_OPS
#define BPF_STRUCT_OPS(name, args...) \
    SEC("struct_ops/"#name) \
    BPF_PROG(name, ##args)
#endif

#ifndef BPF_STRUCT_OPS_SLEEPABLE
#define BPF_STRUCT_OPS_SLEEPABLE(name, args...) \
    SEC("struct_ops.s/"#name) \
    BPF_PROG(name, ##args)
#endif

#ifndef SCX_OPS_DEFINE
#define SCX_OPS_DEFINE(name, ...) \
    SEC(".struct_ops.link") \
    struct sched_ext_ops name = { __VA_ARGS__ };
#endif

/* -----------------------------
 * Kfunc prototypes
 * ----------------------------- */

extern s32 scx_bpf_create_dsq(u64 dsq_id, s32 node) __ksym;
extern void scx_bpf_destroy_dsq(u64 dsq_id) __ksym;
extern u32 scx_bpf_dispatch_nr_slots(void) __ksym;

/* CPU selection helper: keep weak and fallback. */
extern s32 scx_bpf_select_cpu_dfl(struct task_struct *p, s32 prev_cpu,
                                  u64 wake_flags, bool *is_idle) __ksym __weak;

/* Time source. */
extern u64 scx_bpf_now(void) __ksym __weak;
static __always_inline u64 scx_now_ns(void)
{
    if (bpf_ksym_exists(scx_bpf_now))
        return scx_bpf_now();
    return bpf_ktime_get_ns();
}

/* dispatch -> dsq_insert */
extern void scx_bpf_dsq_insert(struct task_struct *p, u64 dsq_id, u64 slice,
                               u64 enq_flags) __ksym __weak;
extern void scx_bpf_dispatch(struct task_struct *p, u64 dsq_id, u64 slice,
                             u64 enq_flags) __ksym __weak;
static __always_inline void scx_insert(struct task_struct *p, u64 dsq_id,
                                       u64 slice, u64 enq_flags)
{
    if (bpf_ksym_exists(scx_bpf_dsq_insert))
        scx_bpf_dsq_insert(p, dsq_id, slice, enq_flags);
    else
        scx_bpf_dispatch(p, dsq_id, slice, enq_flags);
}

/* dispatch_vtime -> dsq_insert_vtime */
extern void scx_bpf_dsq_insert_vtime(struct task_struct *p, u64 dsq_id, u64 slice,
                                     u64 vtime, u64 enq_flags) __ksym __weak;
extern void scx_bpf_dispatch_vtime(struct task_struct *p, u64 dsq_id, u64 slice,
                                   u64 vtime, u64 enq_flags) __ksym __weak;
static __always_inline void scx_insert_vtime(struct task_struct *p, u64 dsq_id,
                                             u64 slice, u64 vtime, u64 enq_flags)
{
    if (bpf_ksym_exists(scx_bpf_dsq_insert_vtime))
        scx_bpf_dsq_insert_vtime(p, dsq_id, slice, vtime, enq_flags);
    else
        scx_bpf_dispatch_vtime(p, dsq_id, slice, vtime, enq_flags);
}

/* consume -> dsq_move_to_local */
extern bool scx_bpf_dsq_move_to_local(u64 dsq_id) __ksym __weak;
extern bool scx_bpf_consume(u64 dsq_id) __ksym __weak;
static __always_inline bool scx_move_to_local(u64 dsq_id)
{
    if (bpf_ksym_exists(scx_bpf_dsq_move_to_local))
        return scx_bpf_dsq_move_to_local(dsq_id);
    return scx_bpf_consume(dsq_id);
}

/* -----------------------------
 * Policy configuration
 * ----------------------------- */

#ifndef SLAM_FULL_SWITCH
#define SLAM_FULL_SWITCH 0
#endif

/* DSQ ids (arbitrary but stable). */
#define DSQ_FE     0xFE01ULL
#define DSQ_BE     0xBE01ULL
#define DSQ_STALE  0x5A1EULL

#define MAX_DISPATCH 32

/* Demote tasks that are already past their deadline by this grace period. */
#define DEADLINE_GRACE_NS 1000000ULL /* 1ms */

/* -----------------------------
 * Maps
 * ----------------------------- */

struct task_state {
    u64 last_start_ns;

    u64 exec_ns_in_job;
    u64 last_job_id;
    u8  overrun; /* budget exceeded for current job */

    u64 vruntime;

    u64 last_reported_deadline_miss_job;
    u64 last_reported_budget_overrun_job;
    u64 last_reported_stale_job;
};

struct {
    __uint(type, BPF_MAP_TYPE_HASH);
    __uint(max_entries, 16384);
    __type(key, u64); /* pid_tgid */
    __type(value, struct slam_task_hint);
} task_hints SEC(".maps");

struct {
    __uint(type, BPF_MAP_TYPE_HASH);
    __uint(max_entries, 16384);
    __type(key, u64); /* pid_tgid */
    __type(value, struct task_state);
} task_states SEC(".maps");

struct {
    __uint(type, BPF_MAP_TYPE_RINGBUF);
    __uint(max_entries, 1 << 20); /* 1 MiB */
} events SEC(".maps");

/* -----------------------------
 * Helpers
 * ----------------------------- */

static __always_inline u64 task_pid_tgid(const struct task_struct *p)
{
    u32 pid = BPF_CORE_READ(p, pid);
    u32 tgid = BPF_CORE_READ(p, tgid);
    return ((u64)tgid << 32) | pid;
}

static __always_inline struct task_state *get_state(u64 key)
{
    struct task_state *st = bpf_map_lookup_elem(&task_states, &key);
    if (st)
        return st;

    struct task_state init = {};
    init.vruntime = scx_now_ns();
    bpf_map_update_elem(&task_states, &key, &init, BPF_NOEXIST);
    return bpf_map_lookup_elem(&task_states, &key);
}

static __always_inline struct slam_task_hint *get_hint(u64 key)
{
    return bpf_map_lookup_elem(&task_hints, &key);
}

static __always_inline void emit_evt(u32 kind, u64 key,
                                     const struct slam_task_hint *h,
                                     const struct task_state *st,
                                     u64 now_ns)
{
    struct slam_evt *e = bpf_ringbuf_reserve(&events, sizeof(*e), 0);
    if (!e)
        return;

    e->ts_ns = now_ns;
    e->pid_tgid = key;

    e->stage_id = h ? h->stage_id : SLAM_STAGE_MISC;
    e->kind = kind;

    e->job_id = h ? h->job_id : 0;
    e->release_ts_ns = h ? h->release_ts_ns : 0;
    e->deadline_ts_ns = h ? h->deadline_ts_ns : 0;

    e->exec_ns_in_job = st ? st->exec_ns_in_job : 0;
    e->age_ns = (h && h->release_ts_ns) ? (now_ns - h->release_ts_ns) : 0;

    e->class_id = h ? h->class_id : SLAM_SCX_CLASS_BE;

    bpf_ringbuf_submit(e, 0);
}

/* Compute effective deadline for FE */
static __always_inline u64 effective_deadline_ns(const struct slam_task_hint *h, u64 now_ns)
{
    u64 eff = h->deadline_ts_ns;

    /* freshness window bounds the effective deadline */
    if (h->stale_ns && h->release_ts_ns) {
        u64 latest_useful = h->release_ts_ns + h->stale_ns;
        if (!eff || latest_useful < eff)
            eff = latest_useful;
    }

    /* no deadline provided => "run soon" */
    if (!eff)
        eff = now_ns;

    return eff;
}

/* -----------------------------
 * sched_ext ops
 * ----------------------------- */

s32 BPF_STRUCT_OPS(scx_slam_fresh_select_cpu, struct task_struct *p,
                   s32 prev_cpu, u64 wake_flags)
{
    bool is_idle = false;

    if (bpf_ksym_exists(scx_bpf_select_cpu_dfl))
        return scx_bpf_select_cpu_dfl(p, prev_cpu, wake_flags, &is_idle);

    return prev_cpu;
}

void BPF_STRUCT_OPS(scx_slam_fresh_enqueue, struct task_struct *p, u64 enq_flags)
{
    const u64 now_ns = scx_now_ns();
    const u64 key = task_pid_tgid(p);

    struct task_state *st = get_state(key);
    struct slam_task_hint *h = get_hint(key);

    if (!st) {
        /* Should be rare (state is created in init_task), but never stall. */
        scx_insert_vtime(p, DSQ_BE, 0, now_ns, enq_flags);
        return;
    }

    /* Default hint: best-effort with no deadlines. */
    struct slam_task_hint dh = {};
    if (!h) {
        dh.api_version = SLAM_SCX_API_VERSION;
        dh.stage_id = SLAM_STAGE_MISC;
        dh.class_id = SLAM_SCX_CLASS_BE;
        dh.job_id = 0;
        dh.release_ts_ns = now_ns;
        h = &dh;
    }

    /* Job boundary => reset per-job accounting. */
    if (h->job_id != st->last_job_id) {
        st->exec_ns_in_job = 0;
        st->overrun = 0;
        st->last_job_id = h->job_id;
    }

    /* Staleness check */
    bool stale = false;
    if (h->stale_ns && h->release_ts_ns) {
        u64 age = now_ns - h->release_ts_ns;
        if (age > h->stale_ns)
            stale = true;
    }

    u64 slice = h->slice_ns; /* 0 => kernel default */

    if (stale) {
        if (st->last_reported_stale_job != h->job_id) {
            emit_evt(SLAM_EVT_STALE_DEMOTION, key, h, st, now_ns);
            st->last_reported_stale_job = h->job_id;
        }
        scx_insert_vtime(p, DSQ_STALE, slice, st->vruntime, enq_flags);
        return;
    }

/* Firm-ish deadline handling: if already past deadline, treat as stale to prevent EDF "doom spiral". */
if (h->deadline_ts_ns && now_ns > h->deadline_ts_ns + DEADLINE_GRACE_NS) {
    if (st->last_reported_deadline_miss_job != h->job_id) {
        emit_evt(SLAM_EVT_DEADLINE_MISS, key, h, st, now_ns);
        st->last_reported_deadline_miss_job = h->job_id;
    }
    scx_insert_vtime(p, DSQ_STALE, slice, st->vruntime, enq_flags);
    return;
}


    if (h->class_id == SLAM_SCX_CLASS_FE && !st->overrun) {
        u64 vtime = effective_deadline_ns(h, now_ns);
        scx_insert_vtime(p, DSQ_FE, slice, vtime, enq_flags);
        return;
    }

    /* BE or FE-overrun => DSQ_BE */
    scx_insert_vtime(p, DSQ_BE, slice, st->vruntime, enq_flags);
}

void BPF_STRUCT_OPS(scx_slam_fresh_dispatch, s32 cpu, struct task_struct *prev)
{
    u32 nr = scx_bpf_dispatch_nr_slots();

#pragma unroll
    for (int i = 0; i < MAX_DISPATCH; i++) {
        if ((u32)i >= nr)
            break;

        if (scx_move_to_local(DSQ_FE))
            continue;
        if (scx_move_to_local(DSQ_BE))
            continue;
        if (scx_move_to_local(DSQ_STALE))
            continue;

        break;
    }
}

void BPF_STRUCT_OPS(scx_slam_fresh_running, struct task_struct *p)
{
    u64 key = task_pid_tgid(p);
    struct task_state *st = get_state(key);
    if (!st)
        return;

    st->last_start_ns = scx_now_ns();
}

void BPF_STRUCT_OPS(scx_slam_fresh_stopping, struct task_struct *p, bool runnable)
{
    const u64 now_ns = scx_now_ns();
    const u64 key = task_pid_tgid(p);

    struct task_state *st = get_state(key);
    struct slam_task_hint *h = get_hint(key);

    if (!st)
        return;

    if (!st->last_start_ns)
        return;

    u64 delta = now_ns - st->last_start_ns;

    st->exec_ns_in_job += delta;
    st->vruntime += delta;

    /* Budget demotion: if exec exceeds budget, mark overrun for this job. */
    if (h && h->budget_ns && st->exec_ns_in_job > h->budget_ns) {
        st->overrun = 1;
        if (st->last_reported_budget_overrun_job != h->job_id) {
            emit_evt(SLAM_EVT_BUDGET_OVERRUN, key, h, st, now_ns);
            st->last_reported_budget_overrun_job = h->job_id;
        }
    }

    /* Deadline miss: best-effort detection (we don't know "job done", only "late while running"). */
    if (h && h->deadline_ts_ns && now_ns > h->deadline_ts_ns) {
        if (st->last_reported_deadline_miss_job != h->job_id) {
            emit_evt(SLAM_EVT_DEADLINE_MISS, key, h, st, now_ns);
            st->last_reported_deadline_miss_job = h->job_id;
        }
    }

    st->last_start_ns = 0;
}

s32 BPF_STRUCT_OPS_SLEEPABLE(scx_slam_fresh_init)
{
    /* NUMA node -1 means "any". */
    s32 err;
    err = scx_bpf_create_dsq(DSQ_FE, -1);
    if (err)
        return err;

    err = scx_bpf_create_dsq(DSQ_BE, -1);
    if (err)
        return err;

    err = scx_bpf_create_dsq(DSQ_STALE, -1);
    if (err)
        return err;

    return 0;
}

void BPF_STRUCT_OPS(scx_slam_fresh_exit, struct scx_exit_info *ei)
{
    scx_bpf_destroy_dsq(DSQ_FE);
    scx_bpf_destroy_dsq(DSQ_BE);
    scx_bpf_destroy_dsq(DSQ_STALE);
}

s32 BPF_STRUCT_OPS_SLEEPABLE(scx_slam_fresh_init_task, struct task_struct *p,
                             struct scx_init_task_args *args)
{
    u64 key = task_pid_tgid(p);
    struct task_state init = {};
    init.vruntime = scx_now_ns();
    bpf_map_update_elem(&task_states, &key, &init, BPF_ANY);
    return 0;
}

void BPF_STRUCT_OPS(scx_slam_fresh_exit_task, struct task_struct *p,
                    struct scx_exit_task_args *args)
{
    u64 key = task_pid_tgid(p);
    bpf_map_delete_elem(&task_states, &key);
}

/*
 * sched_ext ops table.
 *
 * Default: partial switch (safer). Full switch can be selected by building with SLAM_FULL_SWITCH=1.
 *
 * NOTE: The enum constant SCX_OPS_SWITCH_PARTIAL is expected to be present in vmlinux.h
 * when building against a kernel that supports sched_ext.
 */
SCX_OPS_DEFINE(scx_slam_fresh_ops,
    .select_cpu  = (void *)scx_slam_fresh_select_cpu,
    .enqueue     = (void *)scx_slam_fresh_enqueue,
    .dispatch    = (void *)scx_slam_fresh_dispatch,
    .running     = (void *)scx_slam_fresh_running,
    .stopping    = (void *)scx_slam_fresh_stopping,
    .init        = (void *)scx_slam_fresh_init,
    .exit        = (void *)scx_slam_fresh_exit,
    .init_task   = (void *)scx_slam_fresh_init_task,
    .exit_task   = (void *)scx_slam_fresh_exit_task,
    .name        = "scx_slam_fresh",
#if !SLAM_FULL_SWITCH
    .flags       = (u64)SCX_OPS_SWITCH_PARTIAL,
#endif
);

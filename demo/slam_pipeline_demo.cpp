/* SPDX-License-Identifier: MIT */
/*
 * slam_pipeline_demo: synthetic robotics pipeline stress test for scx_slam_fresh.
 *
 * Key idea:
 *   sched_ext makes scheduling decisions at *enqueue time* (wake-up).
 *   If a consumer thread only publishes its "job hint" *after* it runs, the wake-up
 *   is already misclassified, and the demo measures the wrong thing.
 *
 * Therefore this demo uses **wake-safe hinting**:
 *   - each worker thread registers its pid_tgid key (tgid<<32 | tid)
 *   - each queue knows its single consumer's pid_tgid + StageCfg
 *   - a producer publishes the head item's hint before waking a sleeping consumer
 *   - after pop, the consumer republishes the exact item it is processing
 *
 * Pipeline (threads):
 *   IMU propagate (FE, 200Hz)              -> periodic "timer" work
 *   Camera generator (30Hz)               -> vision_frontend (FE) -> state_estimator (FE) -> map_queue
 *   LiDAR generator (10Hz, bursty)        -> lidar_preprocess (FE) -> lidar_registration (BE-ish) -> map_queue
 *   mapping_backend (BE) consumes map_queue (heavy)
 *   hog threads (BE) create contention
 *
 * Options:
 *   --pin <dir>          pinned maps dir from scx_slam_fresh_user
 *   --no-hints           CFS control run without opening BPF maps
 *   --ext-policy <N>     numeric SCHED_EXT policy (7 on Ubuntu 6.14 headers)
 *   --duration <sec>     demo duration (default 10)
 *   --lidar <off|light|mid|heavy>  enable LiDAR stream with point counts
 *   --drop-stale <0|1>   discard expired queued/dequeued jobs (default 0)
 *   --hog <N>            number of hog threads (default 1)
 *   --imu-work-us <N>    IMU compute CPU time per 5ms tick (default 150; 0 disables compute)
 *   --window-stats      separate fixed-window metrics from post-window drain
 *   --camera-burst-count <N>  delay N camera frames, then release together
 *   --camera-burst-at-ms <N>  burst delivery offset (default 3000)
 *   --vision-budget-us <N>     vision FE per-job CPU budget (default 12000)
 *   --vision-work-us <N>       fixed vision FE work; 0 keeps 3-5ms pattern
 *   --vision-deadline-us <N>   vision FE relative deadline (default 33000)
 *   --lidar-pre-budget-us <N>   LiDAR preprocessing CPU budget (default 10000)
 *   --lidar-pre-class <fe|be>   LiDAR preprocessing hint class (default fe)
 *
 * Recommended runs:
 *   # baseline (CFS):
 *   sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar heavy --hog 2 --duration 15
 *
 *   # sched_ext:
 *   sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar heavy --hog 2 --duration 15 --ext-policy 7
 */

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include "window_metrics.h"

#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>

extern "C" {
#include "freshqos.h"
#include "../ros2/scx_slam_executor/include/scx_slam_executor/application_stages.hpp"
#include <linux/sched/types.h>
}

static uint64_t now_ns()
{
    /* CLOCK_MONOTONIC matches bpf_ktime_get_ns() */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* Published once, before the common workload epoch. Zero disables the opt-in
 * instrumentation so existing E0-E3 compute loops retain their old overhead.
 */
static std::atomic<uint64_t> metrics_end_ns{0};

static bool inside_window(uint64_t timestamp)
{
    uint64_t cutoff = metrics_end_ns.load(std::memory_order_relaxed);
    return cutoff && timestamp < cutoff;
}

static uint64_t thread_cpu_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void sleep_until_ns(uint64_t target_ns)
{
    struct timespec ts;
    ts.tv_sec = (time_t)(target_ns / 1000000000ull);
    ts.tv_nsec = (long)(target_ns % 1000000000ull);
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR) {
    }
}

static void busy_work_us(uint64_t us)
{
    uint64_t start = thread_cpu_ns();
    uint64_t dur = us * 1000ULL;
    while (thread_cpu_ns() - start < dur) {
        asm volatile("" ::: "memory");
    }
}

static uint64_t measured_work_us(uint64_t us, WindowStageStats &window)
{
    const uint64_t cutoff = metrics_end_ns.load(std::memory_order_relaxed);
    if (!cutoff) {
        uint64_t start = thread_cpu_ns();
        busy_work_us(us);
        return thread_cpu_ns() - start;
    }

    CpuWindowBounds bounds{cutoff};
    uint64_t before = now_ns();
    const uint64_t start = thread_cpu_ns();
    bounds.sample(0, before, now_ns());
    uint64_t elapsed = 0;
    do {
        if (!bounds.closed) {
            before = now_ns();
            elapsed = thread_cpu_ns() - start;
            bounds.sample(elapsed, before, now_ns());
        } else {
            elapsed = thread_cpu_ns() - start;
        }
        asm volatile("" ::: "memory");
    } while (elapsed < us * 1000ULL);
    bounds.finish(elapsed);
    window.cpu_ns += bounds.lower_ns;
    window.cpu_uncertainty_ns += bounds.upper_ns - bounds.lower_ns;
    return elapsed;
}

static int set_sched_ext_policy(int policy)
{
    if (policy < 0)
        return 0;

    struct sched_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.sched_policy = (uint32_t)policy; /* SCHED_EXT == 7 on Ubuntu headers */
    attr.sched_flags = 0;
    attr.sched_nice = 0;
    attr.sched_priority = 0;

    long rc = syscall(SYS_sched_setattr, 0 /*self*/, &attr, 0);
    if (rc) {
        perror("sched_setattr(SCHED_EXT)");
        _exit(EXIT_FAILURE); /* Never report a mislabeled partial/CFS run. */
    }
    return 0;
}

static void set_thread_name(const char *name)
{
    if (pthread_setname_np(pthread_self(), name) != 0)
        _exit(EXIT_FAILURE); /* Perf attribution must never be silently ambiguous. */
}

struct WorkItem {
    uint64_t seq;
    uint64_t ts_ns;     /* sensor timestamp (release time) */
    uint32_t kind;      /* 1=cam, 2=lidar */
    uint32_t points;    /* lidar only */
    uint32_t burst_id;  /* nonzero when delayed for a synthetic sensor burst */
};

struct StageCfg {
    const char *name;
    uint32_t stage_id;
    uint32_t class_id;           /* FE or BE */
    uint64_t deadline_rel_ns;    /* relative deadline from ts_ns */
    uint64_t stale_ns;           /* freshness window */
    uint64_t budget_ns;          /* per-job budget (0 = no enforcement) */
    uint64_t slice_ns;           /* slice hint (0 = scheduler default) */
    uint32_t weight;             /* BE weight (0 = default) */
};

struct StageStats {
    uint64_t processed{0};
    uint64_t late{0};
    uint64_t cpu_ns{0};
    uint64_t stale_seen{0};
    uint64_t dropped_stale{0};
    uint64_t queue_evicted_stale{0};
    uint64_t burst_processed{0};
    uint64_t burst_latest_seq{0};
    uint64_t burst_latest_age_ns{0};
    uint64_t focus_job{0};
    uint64_t focus_release_ns{0};
    uint64_t focus_completion_ns{0};
    WindowStageStats window;
    int actual_policy{-1};
};

static inline uint64_t deadline_abs(const WorkItem &w, const StageCfg &cfg)
{
    if (!cfg.deadline_rel_ns)
        return 0;
    return w.ts_ns + cfg.deadline_rel_ns;
}

static inline bool is_stale(uint64_t now, const WorkItem &w, const StageCfg &cfg)
{
    if (!cfg.stale_ns)
        return false;
    return (now > w.ts_ns) && (now - w.ts_ns > cfg.stale_ns);
}

struct HintTarget {
    struct freshqos *pub;                 /* shared map fd */
    std::atomic<uint64_t> *pid_tgid;     /* consumer pid_tgid */
    const StageCfg *cfg;                /* consumer scheduling metadata */
};

template <typename T>
class BlockingQueue {
public:
    void set_hint_target(const HintTarget &t)
    {
        ht_ = t;
    }

    void set_stale_eviction(bool enabled)
    {
        std::unique_lock<std::mutex> lk(mu_);
        evict_stale_ = enabled;
    }

    void push(const T &v)
    {
        std::unique_lock<std::mutex> lk(mu_);

        /* A starved consumer cannot dequeue and reject its stale backlog.
         * Let producers prune only expired queued items. The item currently
         * being processed is no longer in q_ and its active hint is untouched.
         */
        uint64_t timestamp = now_ns();
        evict_stale_locked(timestamp);
        if (inside_window(timestamp))
            window_.offered++;

        /* Only publish from the producer when it is about to wake a sleeping
         * consumer. Publishing every queued item would overwrite the hint for
         * an older item that the consumer is still processing.
         */
        if (waiting_) {
            publish_hint(v);
            waiting_ = false;
        }

        q_.push_back(v);
        cv_.notify_one();
    }

    bool pop(T &out)
    {
        std::unique_lock<std::mutex> lk(mu_);
        while (!stop_ && q_.empty()) {
            waiting_ = true;
            cv_.wait(lk);
            waiting_ = false;
        }

        if (stop_)
            return false;

        out = q_.front();
        if (inside_window(now_ns()))
            window_.dequeued++;
        q_.pop_front();

        /* Synchronize the map with the exact FIFO item now being processed.
         * This complements producer publication at the wake-up boundary.
         */
        publish_hint(out);
        return true;
    }

    void stop()
    {
        std::unique_lock<std::mutex> lk(mu_);
        stop_ = true;
        cv_.notify_all();
    }

    uint64_t queue_evicted_stale()
    {
        std::unique_lock<std::mutex> lk(mu_);
        return queue_evicted_stale_;
    }

    size_t pending()
    {
        std::unique_lock<std::mutex> lk(mu_);
        return q_.size();
    }

    WindowQueueStats window_stats()
    {
        std::unique_lock<std::mutex> lk(mu_);
        return window_;
    }

private:
    void evict_stale_locked(uint64_t now)
    {
        if (!evict_stale_ || !ht_.cfg || !ht_.cfg->stale_ns)
            return;

        for (auto it = q_.begin(); it != q_.end();) {
            if (is_stale(now, *it, *ht_.cfg)) {
                it = q_.erase(it);
                queue_evicted_stale_++;
                if (inside_window(now))
                    window_.evicted++;
            } else {
                ++it;
            }
        }
    }

    void publish_hint(const T &v)
    {
        if (!ht_.pub || ht_.pub->map_fd < 0 || !ht_.pid_tgid || !ht_.cfg)
            return;

        uint64_t key = ht_.pid_tgid->load(std::memory_order_acquire);
        if (!key)
            return;

        const StageCfg &c = *ht_.cfg;
        uint64_t dl = c.deadline_rel_ns ? v.ts_ns + c.deadline_rel_ns : 0;
        (void)freshqos_publish_job_for(ht_.pub,
                                     key,
                                     c.stage_id,
                                     c.class_id,
                                     v.seq,
                                     v.ts_ns,
                                     dl,
                                     c.stale_ns,
                                     c.budget_ns,
                                     c.slice_ns,
                                     c.weight);
    }

    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<T> q_;
    bool stop_{false};
    bool waiting_{false};
    bool evict_stale_{false};
    uint64_t queue_evicted_stale_{0};
    WindowQueueStats window_;
    HintTarget ht_{};
};

static void stage_worker(BlockingQueue<WorkItem> *in,
                         BlockingQueue<WorkItem> *out,
                         const StageCfg &cfg,
                         int ext_policy,
                         bool drop_stale,
                         std::atomic<bool> *running,
                         StageStats *stats,
                         const std::function<uint64_t(const WorkItem&)> &compute_us_fn,
                         std::atomic<uint64_t> *pid_out)
{
    set_thread_name(cfg.name);
    (void)set_sched_ext_policy(ext_policy);
    if (pid_out)
        pid_out->store(freshqos_pid_tgid_self(), std::memory_order_release);

    while (running->load()) {
        WorkItem w;
        if (!in->pop(w))
            break;

        uint64_t t0 = now_ns();
        uint64_t dl = deadline_abs(w, cfg);
        bool stale = is_stale(t0, w, cfg);

        if (inside_window(t0)) {
            stats->window.started++;
            if (stale)
                stats->window.stale_seen++;
        }

        if (stale)
            stats->stale_seen++;

        if (stale && drop_stale) {
            stats->dropped_stale++;
            if (inside_window(t0))
                stats->window.dropped_stale++;
            continue;
        }

        uint64_t work_us = compute_us_fn ? compute_us_fn(w) : 0;
        if (work_us) {
            stats->cpu_ns += measured_work_us(work_us, stats->window);
        }

        uint64_t t1 = now_ns();
        stats->processed++;
        if (dl && t1 > dl)
            stats->late++;
        if (inside_window(t1)) {
            stats->window.completed++;
            if (dl && t1 > dl)
                stats->window.late++;
        }

        if (w.seq == 4) {
            stats->focus_job = w.seq;
            stats->focus_release_ns = w.ts_ns;
            stats->focus_completion_ns = t1;
        }

        if (w.burst_id) {
            stats->burst_processed++;
            stats->burst_latest_seq = w.seq;
            stats->burst_latest_age_ns = t1 > w.ts_ns ? t1 - w.ts_ns : 0;
        }

        if (out)
            out->push(w);
    }
}

static void imu_thread(const StageCfg &cfg,
                       struct freshqos *pub,
                       int ext_policy,
                       StageStats *stats,
                       double imu_hz,
                       uint64_t work_us,
                       int duration_s,
                       std::atomic<uint64_t> *workload_start_ns,
                       std::atomic<uint64_t> *pid_out)
{
    /* The optional BPF probe can identify this worker even before its first
     * hint, or if a later hint is missing/misclassified. Not a policy input.
     */
    set_thread_name("imu_prop");
    (void)set_sched_ext_policy(ext_policy);
    stats->actual_policy = sched_getscheduler(0);
    if (stats->actual_policy < 0 || (ext_policy >= 0 && stats->actual_policy != ext_policy))
        _exit(EXIT_FAILURE);

    uint64_t self = freshqos_pid_tgid_self();
    if (pid_out)
        pid_out->store(self, std::memory_order_release);

    const uint64_t period_ns = (uint64_t)(1e9 / imu_hz);
    uint64_t next = 0;
    while (!(next = workload_start_ns->load(std::memory_order_acquire)))
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

    const uint64_t end_ns = next + (uint64_t)duration_s * 1000000000ULL;
    uint64_t seq = 1;

    while (next < end_ns) {
        /* Publish the *next* job hint before sleeping, so wake-up enqueue sees it. */
        if (pub && pub->map_fd >= 0) {
            uint64_t release = next;
            uint64_t dl = (cfg.deadline_rel_ns) ? (release + cfg.deadline_rel_ns) : 0;
            (void)freshqos_publish_job_for(pub,
                                         self,
                                         cfg.stage_id,
                                         cfg.class_id,
                                         seq,
                                         release,
                                         dl,
                                         cfg.stale_ns,
                                         cfg.budget_ns,
                                         cfg.slice_ns,
                                         cfg.weight);
        }

        /* Sleep until the next tick (absolute). */
        sleep_until_ns(next);

        uint64_t dl_abs = (cfg.deadline_rel_ns) ? (next + cfg.deadline_rel_ns) : 0;

        uint64_t t0 = now_ns();
        bool stale = t0 > next && t0 - next > cfg.stale_ns;
        if (stale)
            stats->stale_seen++;
        if (inside_window(t0)) {
            stats->window.started++;
            if (stale)
                stats->window.stale_seen++;
        }
        stats->cpu_ns += measured_work_us(work_us, stats->window);

        uint64_t end = now_ns();
        stats->processed++;
        if (dl_abs && end > dl_abs)
            stats->late++;
        if (inside_window(end)) {
            stats->window.completed++;
            if (dl_abs && end > dl_abs)
                stats->window.late++;
        }

        seq++;
        next += period_ns;

    }
}

static void hog_thread(std::atomic<bool> *running, int ext_policy)
{
    set_thread_name("cpu_hog");
    (void)set_sched_ext_policy(ext_policy);
    while (running->load()) {
        busy_work_us(20000);
    }
}

static uint32_t lidar_points_for_mode(const std::string &mode)
{
    if (mode == "light") return 50000;
    if (mode == "mid")   return 150000;
    if (mode == "heavy") return 300000;
    return 0;
}

static double pct(uint64_t a, uint64_t b)
{
    if (!b) return 0.0;
    return 100.0 * (double)a / (double)b;
}

static uint64_t stale_seen_total(const StageStats &stats)
{
    return stats.stale_seen + stats.queue_evicted_stale;
}

static uint64_t dropped_stale_total(const StageStats &stats)
{
    return stats.dropped_stale + stats.queue_evicted_stale;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s (--pin <dir> | --no-hints) [--ext-policy N] [--duration S] [--lidar off|light|mid|heavy] [--drop-stale 0|1] [--hog N] [--imu-work-us N] [--camera-burst-count N] [--camera-burst-at-ms N] [--vision-budget-us N] [--vision-work-us N] [--vision-deadline-us N] [--lidar-pre-budget-us N] [--lidar-pre-class fe|be]\n"
        "  --imu-work-us N: IMU compute CPU microseconds per 5ms tick (default 150; 0 disables compute)\n"
        "  --lidar-pre-budget-us N: LiDAR preprocessing CPU budget (default 10000)\n"
        "  --lidar-pre-class fe|be: LiDAR preprocessing hint class (default fe)\n"
        "  --window-stats: timestamp fixed-window counters and separate drain metrics\n",
        argv0);
}

static void print_window_stage(const char *name, const StageStats &stats,
                               const WindowQueueStats &queue)
{
    const auto &w = stats.window;
    const uint64_t inflight = queue.dequeued - w.completed - w.dropped_stale;
    printf("window_%s: offered=%llu completed=%llu late=%llu stale_seen=%llu "
           "dropped_stale=%llu pending=%llu in_flight=%llu cpu_us=%llu "
           "cpu_ns=%llu cpu_uncertainty_ns=%llu\n", name,
           (unsigned long long)queue.offered, (unsigned long long)w.completed,
           (unsigned long long)w.late, (unsigned long long)(w.stale_seen + queue.evicted),
           (unsigned long long)(w.dropped_stale + queue.evicted),
           (unsigned long long)queue.pending(), (unsigned long long)inflight,
           (unsigned long long)(w.cpu_ns / 1000), (unsigned long long)w.cpu_ns,
           (unsigned long long)w.cpu_uncertainty_ns);
    /* Window CPU is a lower bound, so its exact complement is a drain upper
     * bound with the same uncertainty. Nanoseconds reconcile without rounding.
     */
    printf("drain_%s: completed=%llu late=%llu cpu_us=%llu cpu_ns=%llu "
           "cpu_uncertainty_ns=%llu\n", name,
           (unsigned long long)(stats.processed - w.completed),
           (unsigned long long)(stats.late - w.late),
           (unsigned long long)((stats.cpu_ns - w.cpu_ns) / 1000),
           (unsigned long long)(stats.cpu_ns - w.cpu_ns),
           (unsigned long long)w.cpu_uncertainty_ns);
}

static void print_focus_job(const char *name, const StageStats &stats,
                            const StageCfg &cfg)
{
    if (stats.focus_job != 4 || !stats.focus_completion_ns)
        return;
    const uint64_t age_ns = stats.focus_completion_ns - stats.focus_release_ns;
    const bool late = cfg.deadline_rel_ns && age_ns > cfg.deadline_rel_ns;
    printf("focus_%s: job=%llu release_ns=%llu completion_ns=%llu age_ns=%llu late=%u\n",
           name, (unsigned long long)stats.focus_job,
           (unsigned long long)stats.focus_release_ns,
           (unsigned long long)stats.focus_completion_ns,
           (unsigned long long)age_ns, (unsigned int)late);
}

static void wait_for_pid(std::atomic<uint64_t> &pid, const char *name)
{
    for (int i = 0; i < 2000; i++) { /* ~2s worst-case */
        if (pid.load(std::memory_order_acquire))
            return;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fprintf(stderr, "warning: timed out waiting for %s pid_tgid\n", name);
}

static double lidar_reg_k_for_mode(const std::string &mode)
{
    if (mode == "light") return 0.01;
    if (mode == "mid")   return 0.017;
    if (mode == "heavy") return 0.05;
    return 0.0; // off/unknown
}

static uint64_t lidar_reg_work_us(uint32_t points, double reg_k)
{
    double n = (double)points;
    double lg = n > 1.0 ? std::log(n) / std::log(2.0) : 1.0;
    double us = 8000.0 + reg_k * n * lg;
    return us > 0.0 ? (uint64_t)us : 0;
}

int main(int argc, char **argv)
{
    const char *pin_dir = nullptr;
    bool no_hints = false;
    bool window_stats = false;
    int ext_policy = -1;
    int duration_s = 10;
    std::string lidar_mode = "off";
    bool drop_stale = false;
    int hog_n = 1;
    uint64_t imu_work_us = 150;
    int camera_burst_count = 0;
    int camera_burst_at_ms = 3000;
    uint64_t vision_budget_us = 12'000;
    uint64_t vision_work_us = 0;
    uint64_t vision_deadline_us = 33'000;
    uint64_t lidar_pre_budget_us = 10'000;
    std::string lidar_pre_class = "fe";

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--pin") && i + 1 < argc) {
            pin_dir = argv[++i];
        } else if (!strcmp(argv[i], "--no-hints")) {
            no_hints = true;
        } else if (!strcmp(argv[i], "--window-stats")) {
            window_stats = true;
        } else if (!strcmp(argv[i], "--ext-policy") && i + 1 < argc) {
            ext_policy = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--duration") && i + 1 < argc) {
            duration_s = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--lidar") && i + 1 < argc) {
            lidar_mode = argv[++i];
        } else if (!strcmp(argv[i], "--drop-stale") && i + 1 < argc) {
            drop_stale = atoi(argv[++i]) != 0;
        } else if (!strcmp(argv[i], "--hog") && i + 1 < argc) {
            hog_n = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--imu-work-us") && i + 1 < argc) {
            const char *arg = argv[++i];
            char *end = nullptr;
            errno = 0;
            unsigned long long value = strtoull(arg, &end, 10);
            if (!*arg || strspn(arg, "0123456789") != strlen(arg) ||
                errno || !end || *end != '\0' || value > UINT64_MAX / 1'000ULL) {
                fprintf(stderr, "error: IMU work must be a non-negative integer number of microseconds that fits in nanoseconds\n");
                return 1;
            }
            imu_work_us = (uint64_t)value;
        } else if (!strcmp(argv[i], "--camera-burst-count") && i + 1 < argc) {
            camera_burst_count = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--camera-burst-at-ms") && i + 1 < argc) {
            camera_burst_at_ms = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--vision-budget-us") && i + 1 < argc) {
            char *end = nullptr;
            errno = 0;
            unsigned long long value = strtoull(argv[++i], &end, 10);
            if (errno || !end || *end != '\0') {
                fprintf(stderr, "error: vision budget must be an integer number of microseconds\n");
                return 1;
            }
            if (value > UINT64_MAX / 1'000ULL) {
                fprintf(stderr, "error: vision budget is too large\n");
                return 1;
            }
            vision_budget_us = (uint64_t)value;
        } else if (!strcmp(argv[i], "--vision-work-us") && i + 1 < argc) {
            char *end = nullptr;
            errno = 0;
            unsigned long long value = strtoull(argv[++i], &end, 10);
            if (errno || !end || *end != '\0') {
                fprintf(stderr, "error: vision work must be an integer number of microseconds\n");
                return 1;
            }
            vision_work_us = (uint64_t)value;
        } else if (!strcmp(argv[i], "--vision-deadline-us") && i + 1 < argc) {
            char *end = nullptr;
            errno = 0;
            unsigned long long value = strtoull(argv[++i], &end, 10);
            if (errno || !end || *end != '\0' || value > UINT64_MAX / 1'000ULL) {
                fprintf(stderr, "error: vision deadline must be a valid number of microseconds\n");
                return 1;
            }
            vision_deadline_us = (uint64_t)value;
        } else if (!strcmp(argv[i], "--lidar-pre-budget-us") && i + 1 < argc) {
            const char *arg = argv[++i];
            char *end = nullptr;
            errno = 0;
            unsigned long long value = strtoull(arg, &end, 10);
            if (!*arg || strspn(arg, "0123456789") != strlen(arg) ||
                errno || !end || *end != '\0' || value > UINT64_MAX / 1'000ULL) {
                fprintf(stderr, "error: LiDAR preprocessing budget must be a non-negative integer number of microseconds that fits in nanoseconds\n");
                return 1;
            }
            lidar_pre_budget_us = (uint64_t)value;
        } else if (!strcmp(argv[i], "--lidar-pre-class") && i + 1 < argc) {
            lidar_pre_class = argv[++i];
            if (lidar_pre_class != "fe" && lidar_pre_class != "be") {
                fprintf(stderr, "error: LiDAR preprocessing class must be fe or be\n");
                return 1;
            }
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if ((!pin_dir && !no_hints) || (pin_dir && no_hints) ||
        (no_hints && ext_policy >= 0)) {
        usage(argv[0]);
        return 1;
    }

    constexpr uint64_t camera_period_ns = 33'000'000ULL;
    if (duration_s <= 0 || hog_n < 0 || camera_burst_count < 0 ||
        camera_burst_at_ms < 0) {
        fprintf(stderr, "error: duration, hog count, and camera burst values must be non-negative (duration must be positive)\n");
        return 1;
    }

    const uint64_t camera_burst_delivery_index =
        ((uint64_t)camera_burst_at_ms * 1'000'000ULL + camera_period_ns - 1) /
        camera_period_ns;
    const uint64_t camera_frame_count =
        ((uint64_t)duration_s * 1'000'000'000ULL + camera_period_ns - 1) /
        camera_period_ns;
    if (camera_burst_count > 0 &&
        (camera_burst_delivery_index >= camera_frame_count ||
         (uint64_t)camera_burst_count > camera_burst_delivery_index + 1)) {
        fprintf(stderr, "error: camera burst must fit between workload start and end\n");
        return 1;
    }

    /* Shared hint publisher used by producers (threads share the fd safely). */
    struct freshqos pub;
    pub.map_fd = -1;
    if (!no_hints && freshqos_open(&pub, pin_dir) != 0) {
        fprintf(stderr, "error: hints are required when --pin is used\n");
        return 1;
    }

    /* Queues */
    BlockingQueue<WorkItem> q_cam;
    BlockingQueue<WorkItem> q_vis;
    BlockingQueue<WorkItem> q_lidar0;
    BlockingQueue<WorkItem> q_lidar1;
    BlockingQueue<WorkItem> q_map;

    /* Stage configuration: slices are left as 0 (scheduler default) to avoid micro-fragmentation. */
    StageCfg cfg_imu { "imu_prop",   SLAM_STAGE_IMU_PREINT,  FRESH_CLASS_URGENT,
                       5'000'000ULL, 10'000'000ULL, 300'000ULL, 0, 0 };

    StageCfg cfg_vis { "vision_fe",  SLAM_STAGE_VISION_FE,   FRESH_CLASS_DEADLINE,
                       vision_deadline_us * 1'000ULL, 66'000'000ULL,
                       vision_budget_us * 1'000ULL, 0, 0 };

    StageCfg cfg_est { "state_est",  SLAM_STAGE_STATE_EST,   FRESH_CLASS_DEADLINE,
                       33'000'000ULL, 66'000'000ULL, 12'000'000ULL, 0, 0 };

    const uint32_t lidar_pre_class_id = lidar_pre_class == "fe" ? FRESH_CLASS_DEADLINE : FRESH_CLASS_BACKGROUND;
    StageCfg cfg_lpre{ "lidar_pre",  SLAM_STAGE_LIDAR_PREINT,lidar_pre_class_id,
                       100'000'000ULL, 150'000'000ULL,
                       lidar_pre_budget_us * 1'000ULL, 0, 0 };

    StageCfg cfg_lreg{ "lidar_reg",  SLAM_STAGE_LIDAR_REG,   FRESH_CLASS_BACKGROUND,
                       200'000'000ULL, 250'000'000ULL, 0, 0, 0 };

    StageCfg cfg_map { "mapping_be", SLAM_STAGE_MAPPING_BE,  FRESH_CLASS_BACKGROUND,
                       400'000'000ULL, 800'000'000ULL, 0, 0, 0 };

    StageStats st_imu, st_vis, st_est, st_lpre, st_lreg, st_map;
    std::atomic<bool> running{true};
    std::atomic<uint64_t> workload_start_ns{0};

    /* Consumer pid_tgid registrations */
    std::atomic<uint64_t> pid_imu{0}, pid_vis{0}, pid_est{0}, pid_lpre{0}, pid_lreg{0}, pid_map{0};

    /* Wire wake-safe hint targets for producer wake-up and consumer pop. */
    q_cam.set_hint_target({ &pub, &pid_vis, &cfg_vis });
    q_vis.set_hint_target({ &pub, &pid_est, &cfg_est });
    q_lidar0.set_hint_target({ &pub, &pid_lpre, &cfg_lpre });
    q_lidar1.set_hint_target({ &pub, &pid_lreg, &cfg_lreg });
    q_map.set_hint_target({ &pub, &pid_map, &cfg_map });

    /* With stale dropping enabled, producers also prune expired queued work.
     * This works even while a consumer is busy or starved and therefore unable
     * to reach the usual dequeue-time stale check.
     */
    q_cam.set_stale_eviction(drop_stale);
    q_vis.set_stale_eviction(drop_stale);
    q_lidar0.set_stale_eviction(drop_stale);
    q_lidar1.set_stale_eviction(drop_stale);
    q_map.set_stale_eviction(drop_stale);

    /* Start workers (consumers). */
    std::thread t_imu(imu_thread, cfg_imu, &pub, ext_policy, &st_imu, 200.0,
                      imu_work_us, duration_s, &workload_start_ns, &pid_imu);

    std::thread t_vis(stage_worker, &q_cam, &q_vis, cfg_vis, ext_policy, drop_stale, &running, &st_vis,
                      [vision_work_us](const WorkItem &w) -> uint64_t {
                          if (vision_work_us)
                              return vision_work_us;
                          return 3000 + (w.seq % 5) * 500; /* 3-5ms */
                      },
                      &pid_vis);

    std::thread t_est(stage_worker, &q_vis, &q_map, cfg_est, ext_policy, drop_stale, &running, &st_est,
                      [](const WorkItem &w) -> uint64_t {
                          return 2000 + (w.seq % 5) * 500; /* 2-4ms */
                      },
                      &pid_est);

    uint32_t lidar_points = lidar_points_for_mode(lidar_mode);
    double lidar_k = lidar_reg_k_for_mode(lidar_mode);

    std::thread t_lpre;
    std::thread t_lreg;
    if (lidar_points) {
        t_lpre = std::thread(stage_worker, &q_lidar0, &q_lidar1, cfg_lpre, ext_policy, drop_stale, &running, &st_lpre,
                             [](const WorkItem &w) -> uint64_t {
                                 /* preprocess ~ O(N): 2ms + 0.05us/point */
                                 return 2000 + (uint64_t)(w.points * 0.05);
                             },
                             &pid_lpre);

        t_lreg = std::thread(stage_worker, &q_lidar1, &q_map, cfg_lreg, ext_policy, drop_stale, &running, &st_lreg,
                             [lidar_k](const WorkItem &w) -> uint64_t {
                                 /* registration ~ O(N log N): 8ms + 0.01us * N * log2(N) */
                                 return lidar_reg_work_us(w.points, lidar_k);
                             },
                             &pid_lreg);
    }

    std::thread t_map(stage_worker, &q_map, nullptr, cfg_map, ext_policy, drop_stale, &running, &st_map,
                      [](const WorkItem &w) -> uint64_t {
                          return 1000 + (w.seq % 5) * 500; /* 1-3ms */
                      },
                      &pid_map);

    /* Hog threads */
    std::vector<std::thread> hogs;
    hogs.reserve((size_t)hog_n);
    for (int i = 0; i < hog_n; i++)
        hogs.emplace_back(hog_thread, &running, ext_policy);

    /* Wait for consumers to publish their pid_tgid before starting generators. */
    wait_for_pid(pid_imu,  "imu_prop");
    wait_for_pid(pid_vis,  "vision_fe");
    wait_for_pid(pid_est,  "state_est");
    wait_for_pid(pid_map,  "mapping_be");
    if (lidar_points) {
        wait_for_pid(pid_lpre, "lidar_pre");
        wait_for_pid(pid_lreg, "lidar_reg");
    }

    const uint64_t workload_start = now_ns() + 100'000'000ULL;
    const uint64_t workload_end = workload_start + (uint64_t)duration_s * 1'000'000'000ULL;
    if (window_stats)
        metrics_end_ns.store(workload_end, std::memory_order_relaxed);
    workload_start_ns.store(workload_start, std::memory_order_release);

    uint64_t cam_generated = 0;
    uint64_t lidar_generated = 0;

    /* Generators run under normal CFS. Absolute release times keep the offered
     * workload identical across policies; a delayed generator creates a burst
     * instead of silently lowering the sensor rate.
     */
    std::thread cam_gen([&]{
        set_thread_name("camera_gen");
        uint64_t seq = 1;
        const uint64_t period_ns = camera_period_ns;
        const uint64_t start = workload_start;
        const uint64_t end = start + (uint64_t)duration_s * 1000000000ULL;
        const uint64_t burst_first_index = camera_burst_count > 0
            ? camera_burst_delivery_index + 1 - (uint64_t)camera_burst_count
            : 0;
        std::vector<WorkItem> held_burst;
        held_burst.reserve((size_t)camera_burst_count);

        uint64_t frame_index = 0;
        for (uint64_t release = start; release < end;
             release += period_ns, frame_index++) {
            sleep_until_ns(release);
            WorkItem w{};
            w.seq = seq++;
            w.ts_ns = release;
            w.kind = 1;
            w.points = 0;
            const bool in_burst = camera_burst_count > 0 &&
                frame_index >= burst_first_index &&
                frame_index <= camera_burst_delivery_index;

            if (in_burst) {
                w.burst_id = 1;
                held_burst.push_back(w);
                if (frame_index == camera_burst_delivery_index) {
                    for (const WorkItem &held : held_burst) {
                        q_cam.push(held);
                        cam_generated++;
                    }
                    held_burst.clear();
                }
            } else {
                q_cam.push(w);
                cam_generated++;
            }
        }
    });

    std::thread lidar_gen([&]{
        set_thread_name("lidar_gen");
        if (!lidar_points)
            return;
        uint64_t seq = 1;
        const uint64_t period_ns = 100'000'000ULL;
        const uint64_t start = workload_start;
        const uint64_t end = start + (uint64_t)duration_s * 1000000000ULL;
        for (uint64_t release = start; release < end; release += period_ns) {
            sleep_until_ns(release);
            WorkItem w{};
            w.seq = seq++;
            w.ts_ns = release;
            w.kind = 2;
            w.points = lidar_points;
            q_lidar0.push(w);
            lidar_generated++;
        }
    });

    cam_gen.join();
    lidar_gen.join();
    t_imu.join();

    if (window_stats)
        sleep_until_ns(workload_end); /* Never end an underloaded window early. */

    running.store(false);

    q_cam.stop();
    q_vis.stop();
    q_lidar0.stop();
    q_lidar1.stop();
    q_map.stop();

    t_vis.join();
    t_est.join();
    if (lidar_points) {
        if (t_lpre.joinable()) t_lpre.join();
        if (t_lreg.joinable()) t_lreg.join();
    }
    t_map.join();
    for (auto &t : hogs) t.join();
    const uint64_t finished_ns = now_ns();

    st_vis.queue_evicted_stale = q_cam.queue_evicted_stale();
    st_est.queue_evicted_stale = q_vis.queue_evicted_stale();
    st_lpre.queue_evicted_stale = q_lidar0.queue_evicted_stale();
    st_lreg.queue_evicted_stale = q_lidar1.queue_evicted_stale();
    st_map.queue_evicted_stale = q_map.queue_evicted_stale();

    const size_t pending_vis = q_cam.pending();
    const size_t pending_est = q_vis.pending();
    const size_t pending_lpre = q_lidar0.pending();
    const size_t pending_lreg = q_lidar1.pending();
    const size_t pending_map = q_map.pending();

    printf("\n=== Results ===\n");
    if (window_stats) {
        printf("measurement: start_ns=%llu end_ns=%llu window_ns=%llu elapsed_ns=%llu drain_elapsed_ns=%llu\n",
               (unsigned long long)workload_start, (unsigned long long)workload_end,
               (unsigned long long)(workload_end - workload_start),
               (unsigned long long)(finished_ns - workload_start),
               (unsigned long long)(finished_ns > workload_end ? finished_ns - workload_end : 0));
        printf("imu_identity: pid_tgid=%llu policy=%d stage_id=%u\n",
               (unsigned long long)pid_imu.load(), st_imu.actual_policy, cfg_imu.stage_id);
        WindowQueueStats imu_queue{(uint64_t)duration_s * 200ULL, st_imu.window.started, 0};
        print_window_stage("imu_prop", st_imu, imu_queue);
        print_window_stage("vision_fe", st_vis, q_cam.window_stats());
        print_window_stage("state_est", st_est, q_vis.window_stats());
        print_focus_job("vision_fe", st_vis, cfg_vis);
        print_focus_job("state_est", st_est, cfg_est);
        if (lidar_points) {
            print_window_stage("lidar_pre", st_lpre, q_lidar0.window_stats());
            print_window_stage("lidar_reg", st_lreg, q_lidar1.window_stats());
        }
        print_window_stage("mapping_be", st_map, q_map.window_stats());
    }
    printf("generated:  imu=%llu camera=%llu lidar=%llu\n",
           (unsigned long long)st_imu.processed,
           (unsigned long long)cam_generated,
           (unsigned long long)lidar_generated);
    printf("camera_burst: injected=%d first_seq=%llu latest_seq=%llu delivery_ms=%llu\n",
           camera_burst_count,
           (unsigned long long)(camera_burst_count > 0
               ? camera_burst_delivery_index + 2 - (uint64_t)camera_burst_count
               : 0),
           (unsigned long long)(camera_burst_count > 0
               ? camera_burst_delivery_index + 1
               : 0),
           (unsigned long long)(camera_burst_count > 0
               ? camera_burst_delivery_index * camera_period_ns / 1'000'000ULL
               : 0));
    printf("configuration: vision_budget_us=%llu vision_work_us=%llu vision_deadline_us=%llu "
           "imu_work_us=%llu lidar_pre_budget_us=%llu lidar_pre_class_id=%u\n",
           (unsigned long long)vision_budget_us,
           (unsigned long long)vision_work_us,
           (unsigned long long)vision_deadline_us,
           (unsigned long long)imu_work_us,
           (unsigned long long)lidar_pre_budget_us, lidar_pre_class_id);
    printf("imu_prop:   processed=%llu late=%llu (%.1f%%) cpu_us=%llu\n",
           (unsigned long long)st_imu.processed,
           (unsigned long long)st_imu.late,
           pct(st_imu.late, st_imu.processed),
           (unsigned long long)(st_imu.cpu_ns / 1000ULL));

    printf("vision_fe:  dequeued=%llu processed=%llu late=%llu (%.1f%%) cpu_us=%llu stale_seen=%llu dropped_stale=%llu consumer_dropped_stale=%llu queue_evicted_stale=%llu pending=%zu burst_processed=%llu burst_latest_seq=%llu burst_latest_age_us=%llu\n",
           (unsigned long long)(st_vis.processed + st_vis.dropped_stale),
           (unsigned long long)st_vis.processed,
           (unsigned long long)st_vis.late,
           pct(st_vis.late, st_vis.processed),
           (unsigned long long)(st_vis.cpu_ns / 1000ULL),
           (unsigned long long)stale_seen_total(st_vis),
           (unsigned long long)dropped_stale_total(st_vis),
           (unsigned long long)st_vis.dropped_stale,
           (unsigned long long)st_vis.queue_evicted_stale,
           pending_vis,
           (unsigned long long)st_vis.burst_processed,
           (unsigned long long)st_vis.burst_latest_seq,
           (unsigned long long)(st_vis.burst_latest_age_ns / 1000ULL));

    printf("state_est:  dequeued=%llu processed=%llu late=%llu (%.1f%%) cpu_us=%llu stale_seen=%llu dropped_stale=%llu consumer_dropped_stale=%llu queue_evicted_stale=%llu pending=%zu burst_processed=%llu burst_latest_seq=%llu burst_latest_age_us=%llu\n",
           (unsigned long long)(st_est.processed + st_est.dropped_stale),
           (unsigned long long)st_est.processed,
           (unsigned long long)st_est.late,
           pct(st_est.late, st_est.processed),
           (unsigned long long)(st_est.cpu_ns / 1000ULL),
           (unsigned long long)stale_seen_total(st_est),
           (unsigned long long)dropped_stale_total(st_est),
           (unsigned long long)st_est.dropped_stale,
           (unsigned long long)st_est.queue_evicted_stale,
           pending_est,
           (unsigned long long)st_est.burst_processed,
           (unsigned long long)st_est.burst_latest_seq,
           (unsigned long long)(st_est.burst_latest_age_ns / 1000ULL));

    if (lidar_points) {
        printf("lidar_pre:  dequeued=%llu processed=%llu late=%llu (%.1f%%) cpu_us=%llu stale_seen=%llu dropped_stale=%llu consumer_dropped_stale=%llu queue_evicted_stale=%llu pending=%zu points=%u\n",
               (unsigned long long)(st_lpre.processed + st_lpre.dropped_stale),
               (unsigned long long)st_lpre.processed,
               (unsigned long long)st_lpre.late,
               pct(st_lpre.late, st_lpre.processed),
               (unsigned long long)(st_lpre.cpu_ns / 1000ULL),
               (unsigned long long)stale_seen_total(st_lpre),
               (unsigned long long)dropped_stale_total(st_lpre),
               (unsigned long long)st_lpre.dropped_stale,
               (unsigned long long)st_lpre.queue_evicted_stale,
               pending_lpre,
               lidar_points);

        printf("lidar_reg:  dequeued=%llu processed=%llu late=%llu (%.1f%%) cpu_us=%llu stale_seen=%llu dropped_stale=%llu consumer_dropped_stale=%llu queue_evicted_stale=%llu pending=%zu points=%u reg_k=%.3f reg_job_us=%llu\n",
               (unsigned long long)(st_lreg.processed + st_lreg.dropped_stale),
               (unsigned long long)st_lreg.processed,
               (unsigned long long)st_lreg.late,
               pct(st_lreg.late, st_lreg.processed),
               (unsigned long long)(st_lreg.cpu_ns / 1000ULL),
               (unsigned long long)stale_seen_total(st_lreg),
               (unsigned long long)dropped_stale_total(st_lreg),
               (unsigned long long)st_lreg.dropped_stale,
               (unsigned long long)st_lreg.queue_evicted_stale,
               pending_lreg,
               lidar_points,
               lidar_k,
               (unsigned long long)lidar_reg_work_us(lidar_points, lidar_k));
    }

    printf("mapping_be: dequeued=%llu processed=%llu late=%llu (%.1f%%) cpu_us=%llu stale_seen=%llu dropped_stale=%llu consumer_dropped_stale=%llu queue_evicted_stale=%llu pending=%zu\n",
           (unsigned long long)(st_map.processed + st_map.dropped_stale),
           (unsigned long long)st_map.processed,
           (unsigned long long)st_map.late,
           pct(st_map.late, st_map.processed),
           (unsigned long long)(st_map.cpu_ns / 1000ULL),
           (unsigned long long)stale_seen_total(st_map),
           (unsigned long long)dropped_stale_total(st_map),
           (unsigned long long)st_map.dropped_stale,
           (unsigned long long)st_map.queue_evicted_stale,
           pending_map);

    printf("\nTry deadline isolation on one CPU:\n");
    printf("  taskset -c 0 ./build/slam_pipeline_demo --no-hints --lidar heavy --hog 2 --duration %d\n", duration_s);
    if (pin_dir)
        printf("  sudo taskset -c 0 ./build/slam_pipeline_demo --pin %s --lidar heavy --hog 2 --duration %d --ext-policy 7\n", pin_dir, duration_s);
    printf("Test stale shedding separately with --hog 0, with and without --drop-stale 1.\n");
    printf("Test burst recovery with --camera-burst-count 12 --camera-burst-at-ms 3000.\n");
    printf("Test budget demotion with --vision-work-us 12000 --vision-deadline-us 30000 --vision-budget-us 1000.\n");

    freshqos_close(&pub);
    return 0;
}

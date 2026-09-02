/* SPDX-License-Identifier: GPL-2.0 */
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
 *   --camera-burst-count <N>  delay N camera frames, then release together
 *   --camera-burst-at-ms <N>  burst delivery offset (default 3000)
 *   --vision-budget-us <N>     vision FE per-job CPU budget (default 12000)
 *   --vision-work-us <N>       fixed vision FE work; 0 keeps 3-5ms pattern
 *   --vision-deadline-us <N>   vision FE relative deadline (default 33000)
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

#include <sys/syscall.h>
#include <unistd.h>
#include <time.h>

extern "C" {
#include "../src/slamqos.h"
#include <linux/sched/types.h>
}

static uint64_t now_ns()
{
    /* CLOCK_MONOTONIC matches bpf_ktime_get_ns() */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
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
    struct slamqos *pub;                 /* shared map fd */
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
        evict_stale_locked(now_ns());

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

private:
    void evict_stale_locked(uint64_t now)
    {
        if (!evict_stale_ || !ht_.cfg || !ht_.cfg->stale_ns)
            return;

        for (auto it = q_.begin(); it != q_.end();) {
            if (is_stale(now, *it, *ht_.cfg)) {
                it = q_.erase(it);
                queue_evicted_stale_++;
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
        (void)slamqos_publish_job_for(ht_.pub,
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
    (void)set_sched_ext_policy(ext_policy);
    if (pid_out)
        pid_out->store(slamqos_pid_tgid_self(), std::memory_order_release);

    while (running->load()) {
        WorkItem w;
        if (!in->pop(w))
            break;

        uint64_t t0 = now_ns();
        uint64_t dl = deadline_abs(w, cfg);
        bool stale = is_stale(t0, w, cfg);

        if (stale)
            stats->stale_seen++;

        if (stale && drop_stale) {
            stats->dropped_stale++;
            continue;
        }

        uint64_t work_us = compute_us_fn ? compute_us_fn(w) : 0;
        if (work_us) {
            uint64_t cpu_start_ns = thread_cpu_ns();
            busy_work_us(work_us);
            stats->cpu_ns += thread_cpu_ns() - cpu_start_ns;
        }

        uint64_t t1 = now_ns();
        stats->processed++;
        if (dl && t1 > dl)
            stats->late++;

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
                       struct slamqos *pub,
                       int ext_policy,
                       StageStats *stats,
                       double imu_hz,
                       int duration_s,
                       std::atomic<uint64_t> *workload_start_ns,
                       std::atomic<uint64_t> *pid_out)
{
    (void)set_sched_ext_policy(ext_policy);

    uint64_t self = slamqos_pid_tgid_self();
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
            (void)slamqos_publish_job_for(pub,
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

        uint64_t cpu_start_ns = thread_cpu_ns();
        busy_work_us(150); /* ~0.15ms */
        stats->cpu_ns += thread_cpu_ns() - cpu_start_ns;

        uint64_t end = now_ns();
        stats->processed++;
        if (dl_abs && end > dl_abs)
            stats->late++;

        seq++;
        next += period_ns;

    }
}

static void hog_thread(std::atomic<bool> *running, int ext_policy)
{
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
        "Usage: %s (--pin <dir> | --no-hints) [--ext-policy N] [--duration S] [--lidar off|light|mid|heavy] [--drop-stale 0|1] [--hog N] [--camera-burst-count N] [--camera-burst-at-ms N] [--vision-budget-us N] [--vision-work-us N] [--vision-deadline-us N]\n",
        argv0);
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
    int ext_policy = -1;
    int duration_s = 10;
    std::string lidar_mode = "off";
    bool drop_stale = false;
    int hog_n = 1;
    int camera_burst_count = 0;
    int camera_burst_at_ms = 3000;
    uint64_t vision_budget_us = 12'000;
    uint64_t vision_work_us = 0;
    uint64_t vision_deadline_us = 33'000;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--pin") && i + 1 < argc) {
            pin_dir = argv[++i];
        } else if (!strcmp(argv[i], "--no-hints")) {
            no_hints = true;
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
    struct slamqos pub;
    pub.map_fd = -1;
    if (!no_hints && slamqos_open(&pub, pin_dir) != 0) {
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
    StageCfg cfg_imu { "imu_prop",   SLAM_STAGE_IMU_PREINT,  SLAM_SCX_CLASS_FE,
                       5'000'000ULL, 10'000'000ULL, 300'000ULL, 0, 0 };

    StageCfg cfg_vis { "vision_fe",  SLAM_STAGE_VISION_FE,   SLAM_SCX_CLASS_FE,
                       vision_deadline_us * 1'000ULL, 66'000'000ULL,
                       vision_budget_us * 1'000ULL, 0, 0 };

    StageCfg cfg_est { "state_est",  SLAM_STAGE_STATE_EST,   SLAM_SCX_CLASS_FE,
                       33'000'000ULL, 66'000'000ULL, 12'000'000ULL, 0, 0 };

    StageCfg cfg_lpre{ "lidar_pre",  SLAM_STAGE_LIDAR_PREINT,SLAM_SCX_CLASS_FE,
                       100'000'000ULL, 150'000'000ULL, 10'000'000ULL, 0, 0 };

    StageCfg cfg_lreg{ "lidar_reg",  SLAM_STAGE_LIDAR_REG,   SLAM_SCX_CLASS_BE,
                       200'000'000ULL, 250'000'000ULL, 0, 0, 0 };

    StageCfg cfg_map { "mapping_be", SLAM_STAGE_MAPPING_BE,  SLAM_SCX_CLASS_BE,
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
                      duration_s, &workload_start_ns, &pid_imu);

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
    workload_start_ns.store(workload_start, std::memory_order_release);

    uint64_t cam_generated = 0;
    uint64_t lidar_generated = 0;

    /* Generators run under normal CFS. Absolute release times keep the offered
     * workload identical across policies; a delayed generator creates a burst
     * instead of silently lowering the sensor rate.
     */
    std::thread cam_gen([&]{
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
    printf("configuration: vision_budget_us=%llu vision_work_us=%llu vision_deadline_us=%llu\n",
           (unsigned long long)vision_budget_us,
           (unsigned long long)vision_work_us,
           (unsigned long long)vision_deadline_us);
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

    slamqos_close(&pub);
    return 0;
}

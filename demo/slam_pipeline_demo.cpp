/* SPDX-License-Identifier: GPL-2.0 */
/*
 * slam_pipeline_demo: synthetic robotics pipeline stress test for scx_slam_fresh.
 *
 * Key idea:
 *   sched_ext makes scheduling decisions at *enqueue time* (wake-up).
 *   If a consumer thread only publishes its "job hint" *after* it runs, the wake-up
 *   is already misclassified, and the demo measures the wrong thing.
 *
 * Therefore this demo uses **producer-driven hinting**:
 *   - each worker thread registers its pid_tgid key (tgid<<32 | tid)
 *   - each queue knows its single consumer's pid_tgid + StageCfg
 *   - when a producer pushes a WorkItem, it publishes the *consumer's* hint first,
 *     then wakes the consumer.
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
 *   --ext-policy <N>     numeric SCHED_EXT policy (7 on Ubuntu 6.14 headers)
 *   --duration <sec>     demo duration (default 10)
 *   --lidar <off|light|mid|heavy>  enable LiDAR stream with point counts
 *   --drop-stale <0|1>   skip compute for stale jobs (default 0)
 *   --hog <N>            number of hog threads (default 1)
 *
 * Recommended runs:
 *   # baseline (CFS):
 *   sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar heavy --hog 2 --duration 15
 *
 *   # sched_ext:
 *   sudo taskset -c 0 ./build/slam_pipeline_demo --pin /sys/fs/bpf/scx_slam_fresh --lidar heavy --hog 2 --duration 15 --ext-policy 7
 */

#include <atomic>
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

static void busy_work_us(uint64_t us)
{
    uint64_t start = now_ns();
    uint64_t dur = us * 1000ULL;
    while (now_ns() - start < dur) {
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
        return -1;
    }
    return 0;
}

struct WorkItem {
    uint64_t seq;
    uint64_t ts_ns;     /* sensor timestamp (release time) */
    uint32_t kind;      /* 1=cam, 2=lidar */
    uint32_t points;    /* lidar only */
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
    uint64_t stale_seen{0};
    uint64_t dropped_stale{0};
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

    void push(const T &v)
    {
        /* Producer-driven hinting:
         * publish the *consumer* hint first, then wake it.
         */
        if (ht_.pub && ht_.pub->map_fd >= 0 && ht_.pid_tgid && ht_.cfg) {
            uint64_t key = ht_.pid_tgid->load(std::memory_order_acquire);
            if (key) {
                const StageCfg &c = *ht_.cfg;
                uint64_t dl = (c.deadline_rel_ns) ? (v.ts_ns + c.deadline_rel_ns) : 0;
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
        }

        std::unique_lock<std::mutex> lk(mu_);
        q_.push_back(v);
        cv_.notify_one();
    }

    bool pop(T &out)
    {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [&]{ return stop_ || !q_.empty(); });
        if (stop_)
            return false;
        out = q_.front();
        q_.pop_front();
        return true;
    }

    void stop()
    {
        std::unique_lock<std::mutex> lk(mu_);
        stop_ = true;
        cv_.notify_all();
    }

private:
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<T> q_;
    bool stop_{false};
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
        if (work_us)
            busy_work_us(work_us);

        uint64_t t1 = now_ns();
        stats->processed++;
        if (dl && t1 > dl)
            stats->late++;

        if (out)
            out->push(w);
    }
}

static void imu_thread(const StageCfg &cfg,
                       struct slamqos *pub,
                       int ext_policy,
                       std::atomic<bool> *running,
                       StageStats *stats,
                       double imu_hz,
                       std::atomic<uint64_t> *pid_out)
{
    (void)set_sched_ext_policy(ext_policy);

    uint64_t self = slamqos_pid_tgid_self();
    if (pid_out)
        pid_out->store(self, std::memory_order_release);

    const uint64_t period_ns = (uint64_t)(1e9 / imu_hz);
    uint64_t next = now_ns();
    uint64_t seq = 1;

    while (running->load()) {
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
        struct timespec ts;
        ts.tv_sec = (time_t)(next / 1000000000ull);
        ts.tv_nsec = (long)(next % 1000000000ull);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

        uint64_t start = now_ns();
        uint64_t dl_abs = (cfg.deadline_rel_ns) ? (next + cfg.deadline_rel_ns) : 0;

        busy_work_us(150); /* ~0.15ms */

        uint64_t end = now_ns();
        stats->processed++;
        if (dl_abs && end > dl_abs)
            stats->late++;

        seq++;
        next += period_ns;

        /* If we slipped a lot, resync. */
        if (end > next + period_ns)
            next = end + period_ns;
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

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s --pin <dir> [--ext-policy N] [--duration S] [--lidar off|light|mid|heavy] [--drop-stale 0|1] [--hog N]\n",
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

int main(int argc, char **argv)
{
    const char *pin_dir = nullptr;
    int ext_policy = -1;
    int duration_s = 10;
    std::string lidar_mode = "off";
    bool drop_stale = false;
    int hog_n = 1;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--pin") && i + 1 < argc) {
            pin_dir = argv[++i];
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
        } else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (!pin_dir) {
        usage(argv[0]);
        return 1;
    }

    /* Shared hint publisher used by producers (threads share the fd safely). */
    struct slamqos pub;
    pub.map_fd = -1;
    if (slamqos_open(&pub, pin_dir) != 0) {
        fprintf(stderr, "warning: failed to open slamqos maps; running without hints\n");
        pub.map_fd = -1;
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
                       33'000'000ULL, 66'000'000ULL, 12'000'000ULL, 0, 0 };

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

    /* Consumer pid_tgid registrations */
    std::atomic<uint64_t> pid_imu{0}, pid_vis{0}, pid_est{0}, pid_lpre{0}, pid_lreg{0}, pid_map{0};

    /* Wire producer-driven hint targets: each queue publishes hints for its consumer. */
    q_cam.set_hint_target({ &pub, &pid_vis, &cfg_vis });
    q_vis.set_hint_target({ &pub, &pid_est, &cfg_est });
    q_lidar0.set_hint_target({ &pub, &pid_lpre, &cfg_lpre });
    q_lidar1.set_hint_target({ &pub, &pid_lreg, &cfg_lreg });
    q_map.set_hint_target({ &pub, &pid_map, &cfg_map });

    /* Start workers (consumers). */
    std::thread t_imu(imu_thread, cfg_imu, &pub, ext_policy, &running, &st_imu, 200.0, &pid_imu);

    std::thread t_vis(stage_worker, &q_cam, &q_vis, cfg_vis, ext_policy, drop_stale, &running, &st_vis,
                      [](const WorkItem &w) -> uint64_t {
                          return 6000 + (w.seq % 5) * 1000; /* 6-10ms */
                      },
                      &pid_vis);

    std::thread t_est(stage_worker, &q_vis, &q_map, cfg_est, ext_policy, drop_stale, &running, &st_est,
                      [](const WorkItem &w) -> uint64_t {
                          return 5000 + (w.seq % 5) * 1000; /* 5-9ms */
                      },
                      &pid_est);

    uint32_t lidar_points = lidar_points_for_mode(lidar_mode);

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
                             [](const WorkItem &w) -> uint64_t {
                                 /* registration ~ O(N log N): 8ms + 0.01us * N * log2(N) */
                                 double N = (double)w.points;
                                 double lg = (N > 1.0) ? (std::log(N) / std::log(2.0)) : 1.0;
                                 double us = 8000.0 + 0.01 * N * lg;
                                 if (us < 0.0) us = 0.0;
                                 return (uint64_t)us;
                             },
                             &pid_lreg);
    }

    std::thread t_map(stage_worker, &q_map, nullptr, cfg_map, ext_policy, drop_stale, &running, &st_map,
                      [](const WorkItem &w) -> uint64_t {
                          (void)w;
                          return 20000 + (w.seq % 9) * 5000; /* 20-60ms */
                      },
                      &pid_map);

    /* Hog threads */
    std::vector<std::thread> hogs;
    hogs.reserve((size_t)hog_n);
    for (int i = 0; i < hog_n; i++)
        hogs.emplace_back(hog_thread, &running, ext_policy);

    /* Wait for consumers to publish their pid_tgid before starting generators. */
    wait_for_pid(pid_vis,  "vision_fe");
    wait_for_pid(pid_est,  "state_est");
    wait_for_pid(pid_map,  "mapping_be");
    if (lidar_points) {
        wait_for_pid(pid_lpre, "lidar_pre");
        wait_for_pid(pid_lreg, "lidar_reg");
    }

    /* Generators run under normal CFS; they just wake the pipeline. */
    std::thread cam_gen([&]{
        uint64_t seq = 1;
        uint64_t start = now_ns();
        while (now_ns() - start < (uint64_t)duration_s * 1000000000ull) {
            WorkItem w{};
            w.seq = seq++;
            w.ts_ns = now_ns();
            w.kind = 1;
            w.points = 0;
            q_cam.push(w);
            std::this_thread::sleep_for(std::chrono::milliseconds(33)); /* 30Hz */
        }
    });

    std::thread lidar_gen([&]{
        if (!lidar_points)
            return;
        uint64_t seq = 1;
        uint64_t start = now_ns();
        while (now_ns() - start < (uint64_t)duration_s * 1000000000ull) {
            WorkItem w{};
            w.seq = seq++;
            w.ts_ns = now_ns();
            w.kind = 2;
            w.points = lidar_points;
            q_lidar0.push(w);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); /* 10Hz */
        }
    });

    cam_gen.join();
    lidar_gen.join();

    running.store(false);

    q_cam.stop();
    q_vis.stop();
    q_lidar0.stop();
    q_lidar1.stop();
    q_map.stop();

    t_imu.join();
    t_vis.join();
    t_est.join();
    if (lidar_points) {
        if (t_lpre.joinable()) t_lpre.join();
        if (t_lreg.joinable()) t_lreg.join();
    }
    t_map.join();
    for (auto &t : hogs) t.join();

    printf("\n=== Results ===\n");
    printf("imu_prop:   processed=%llu late=%llu (%.1f%%)\n",
           (unsigned long long)st_imu.processed,
           (unsigned long long)st_imu.late,
           pct(st_imu.late, st_imu.processed));

    printf("vision_fe:  processed=%llu late=%llu (%.1f%%) stale_seen=%llu dropped_stale=%llu\n",
           (unsigned long long)st_vis.processed,
           (unsigned long long)st_vis.late,
           pct(st_vis.late, st_vis.processed),
           (unsigned long long)st_vis.stale_seen,
           (unsigned long long)st_vis.dropped_stale);

    printf("state_est:  processed=%llu late=%llu (%.1f%%) stale_seen=%llu dropped_stale=%llu\n",
           (unsigned long long)st_est.processed,
           (unsigned long long)st_est.late,
           pct(st_est.late, st_est.processed),
           (unsigned long long)st_est.stale_seen,
           (unsigned long long)st_est.dropped_stale);

    if (lidar_points) {
        printf("lidar_pre:  processed=%llu late=%llu (%.1f%%) stale_seen=%llu dropped_stale=%llu points=%u\n",
               (unsigned long long)st_lpre.processed,
               (unsigned long long)st_lpre.late,
               pct(st_lpre.late, st_lpre.processed),
               (unsigned long long)st_lpre.stale_seen,
               (unsigned long long)st_lpre.dropped_stale,
               lidar_points);

        printf("lidar_reg:  processed=%llu late=%llu (%.1f%%) stale_seen=%llu dropped_stale=%llu points=%u\n",
               (unsigned long long)st_lreg.processed,
               (unsigned long long)st_lreg.late,
               pct(st_lreg.late, st_lreg.processed),
               (unsigned long long)st_lreg.stale_seen,
               (unsigned long long)st_lreg.dropped_stale,
               lidar_points);
    }

    printf("mapping_be: processed=%llu late=%llu (%.1f%%) stale_seen=%llu dropped_stale=%llu\n",
           (unsigned long long)st_map.processed,
           (unsigned long long)st_map.late,
           pct(st_map.late, st_map.processed),
           (unsigned long long)st_map.stale_seen,
           (unsigned long long)st_map.dropped_stale);

    printf("\nTip: compare runs with/without scx_slam_fresh.\n");
    printf("Try single core:\n");
    printf("  sudo taskset -c 0 ./build/slam_pipeline_demo --pin %s --lidar heavy --hog 2 --duration %d\n", pin_dir, duration_s);
    printf("  sudo taskset -c 0 ./build/slam_pipeline_demo --pin %s --lidar heavy --hog 2 --duration %d --ext-policy 7\n", pin_dir, duration_s);
    printf("Add --drop-stale 1 to see wasted-work shedding.\n");

    slamqos_close(&pub);
    return 0;
}

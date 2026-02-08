/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Synthetic SLAM/VIO-ish pipeline demo.
 *
 * Stages:
 *  - vision_frontend (FE)
 *  - state_estimator (FE)
 *  - mapping_backend (BE)
 * Plus a CPU hog thread to create contention.
 *
 * Each stage publishes sched hints via libslamqos.
 */
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <sys/syscall.h>
#include <unistd.h>

extern "C" {
#include "../src/slamqos.h"
#include <linux/sched/types.h>
}

static uint64_t now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

struct Msg {
    uint64_t seq;
    uint64_t ts_ns; /* release time */
};

template <typename T>
class BlockingQueue {
public:
    void push(const T &v) {
        std::unique_lock<std::mutex> lk(mu_);
        q_.push_back(v);
        cv_.notify_one();
    }

    bool pop(T &out) {
        std::unique_lock<std::mutex> lk(mu_);
        cv_.wait(lk, [&]{ return stop_ || !q_.empty(); });
        if (stop_)
            return false;
        out = q_.front();
        q_.pop_front();
        return true;
    }

    void stop() {
        std::unique_lock<std::mutex> lk(mu_);
        stop_ = true;
        cv_.notify_all();
    }

    size_t size() const {
        std::unique_lock<std::mutex> lk(mu_);
        return q_.size();
    }

private:
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<T> q_;
    bool stop_{false};
};

static int set_sched_ext_policy(int policy)
{
    if (policy < 0)
        return 0;

    /* sched_setattr syscall; uses uapi linux/sched/types.h */
    struct sched_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.size = sizeof(attr);
    attr.sched_policy = (uint32_t)policy;
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

static void busy_work_us(uint64_t us)
{
    uint64_t start = now_ns();
    uint64_t dur = us * 1000ULL;
    while (now_ns() - start < dur) {
        asm volatile("" ::: "memory");
    }
}

struct StageCfg {
    uint32_t stage_id;
    uint32_t class_id;
    uint64_t deadline_rel_ns;
    uint64_t stale_ns;
    uint64_t budget_ns;
    uint64_t slice_ns;
};

struct StageStats {
    uint64_t processed{0};
    uint64_t late{0};
    uint64_t stale{0};
};

static void stage_thread(const char *name,
                         BlockingQueue<Msg> *in,
                         BlockingQueue<Msg> *out,
                         const StageCfg &cfg,
                         const char *pin_dir,
                         int ext_policy,
                         std::atomic<bool> *running,
                         StageStats *stats)
{
    /* Optional: put this thread into SCHED_EXT when running in partial switch mode. */
    (void)set_sched_ext_policy(ext_policy);

    struct slamqos q;
    if (slamqos_open(&q, pin_dir) != 0) {
        fprintf(stderr, "[%s] failed to open slamqos; running without hints\n", name);
        q.map_fd = -1;
    }

    while (running->load()) {
        Msg m;
        if (!in->pop(m))
            break;

        uint64_t t0 = now_ns();
        uint64_t deadline = m.ts_ns + cfg.deadline_rel_ns;
        uint64_t age = t0 - m.ts_ns;

        if (cfg.stale_ns && age > cfg.stale_ns) {
            stats->stale++;
        }

        if (q.map_fd >= 0) {
            slamqos_publish_job(&q,
                               cfg.stage_id,
                               cfg.class_id,
                               m.seq,
                               m.ts_ns,
                               deadline,
                               cfg.stale_ns,
                               cfg.budget_ns,
                               cfg.slice_ns,
                               0 /*weight*/);
        }

        /* Simulate compute:
         * - FE stages: a few ms
         * - BE stage: heavy, variable
         */
        if (cfg.class_id == SLAM_SCX_CLASS_FE) {
            busy_work_us(3000 + (m.seq % 5) * 500); /* 3.0ms .. 5.0ms */
        } else {
            busy_work_us(20000 + (m.seq % 7) * 5000); /* 20ms .. 50ms */
        }

        uint64_t t1 = now_ns();
        stats->processed++;
        if (t1 > deadline)
            stats->late++;

        if (out)
            out->push(m);
    }

    slamqos_close(&q);
}

static void hog_thread(std::atomic<bool> *running, int ext_policy)
{
    (void)set_sched_ext_policy(ext_policy);
    while (running->load()) {
        busy_work_us(2000);
        std::this_thread::yield();
    }
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "Usage: %s --pin <dir> [--ext-policy N]\n"
        "\n"
        "  --pin <dir>        directory where scx_slam_fresh_user pinned maps\n"
        "  --ext-policy <N>   numeric policy value for SCHED_EXT (needed in partial switch mode)\n"
        "\n"
        "Example:\n"
        "  sudo %s --pin /sys/fs/bpf/scx_slam_fresh --ext-policy <N>\n",
        argv0, argv0);
}

int main(int argc, char **argv)
{
    const char *pin_dir = nullptr;
    int ext_policy = -1;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--pin") && i + 1 < argc) {
            pin_dir = argv[++i];
        } else if (!strcmp(argv[i], "--ext-policy") && i + 1 < argc) {
            ext_policy = atoi(argv[++i]);
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

    BlockingQueue<Msg> q1, q2, q3;

    StageCfg fe1 { SLAM_STAGE_VISION_FE, SLAM_SCX_CLASS_FE,
                   33'000'000ULL, /* 33ms */
                   66'000'000ULL, /* stale after 66ms */
                   8'000'000ULL,  /* budget 8ms */
                   1'000'000ULL   /* slice hint 1ms */
    };

    StageCfg fe2 { SLAM_STAGE_STATE_EST, SLAM_SCX_CLASS_FE,
                   33'000'000ULL,
                   66'000'000ULL,
                   8'000'000ULL,
                   1'000'000ULL
    };

    StageCfg be  { SLAM_STAGE_MAPPING_BE, SLAM_SCX_CLASS_BE,
                   200'000'000ULL, /* 200ms */
                   500'000'000ULL, /* stale after 500ms */
                   0,              /* no budget */
                   5'000'000ULL    /* 5ms slice hint */
    };

    std::atomic<bool> running{true};
    StageStats st_fe1, st_fe2, st_be;

    std::thread t_fe1(stage_thread, "vision_fe", &q1, &q2, fe1, pin_dir,
                      ext_policy, &running, &st_fe1);
    std::thread t_fe2(stage_thread, "state_est", &q2, &q3, fe2, pin_dir,
                      ext_policy, &running, &st_fe2);
    std::thread t_be(stage_thread, "mapping_be", &q3, nullptr, be, pin_dir,
                     ext_policy, &running, &st_be);

    /* A CPU hog to stress the system. */
    std::thread t_hog(hog_thread, &running, ext_policy);

    /* Sensor generator: 30Hz camera-like messages for 10 seconds. */
    uint64_t seq = 1;
    uint64_t start = now_ns();
    while (now_ns() - start < 10ULL * 1000ULL * 1000ULL * 1000ULL) {
        Msg m;
        m.seq = seq++;
        m.ts_ns = now_ns();
        q1.push(m);
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }

    running.store(false);
    q1.stop(); q2.stop(); q3.stop();

    t_fe1.join();
    t_fe2.join();
    t_be.join();
    t_hog.join();

    auto pct = [](uint64_t a, uint64_t b) -> double {
        if (!b) return 0.0;
        return 100.0 * (double)a / (double)b;
    };

    printf("\n=== Results ===\n");
    printf("vision_fe:  processed=%llu late=%llu (%.1f%%) stale_seen=%llu\n",
           (unsigned long long)st_fe1.processed,
           (unsigned long long)st_fe1.late,
           pct(st_fe1.late, st_fe1.processed),
           (unsigned long long)st_fe1.stale);

    printf("state_est:  processed=%llu late=%llu (%.1f%%) stale_seen=%llu\n",
           (unsigned long long)st_fe2.processed,
           (unsigned long long)st_fe2.late,
           pct(st_fe2.late, st_fe2.processed),
           (unsigned long long)st_fe2.stale);

    printf("mapping_be: processed=%llu late=%llu (%.1f%%) stale_seen=%llu\n",
           (unsigned long long)st_be.processed,
           (unsigned long long)st_be.late,
           pct(st_be.late, st_be.processed),
           (unsigned long long)st_be.stale);

    printf("\nTip: run with/without scx_slam_fresh to compare late rates.\n");
    return 0;
}

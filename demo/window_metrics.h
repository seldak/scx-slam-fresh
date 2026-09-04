/* SPDX-License-Identifier: GPL-2.0 */
#pragma once

#include <cstdint>

/* CPU and wall clocks cannot be read atomically. Bracket each CPU read with
 * monotonic wall reads. At the cutoff, retain a conservative CPU lower bound
 * and the first provable upper bound; never interpolate across preemption.
 */
struct CpuWindowBounds {
    uint64_t cutoff_ns;
    uint64_t lower_ns{0};
    uint64_t upper_ns{0};
    bool closed{false};

    void sample(uint64_t cpu_elapsed, uint64_t wall_before, uint64_t wall_after)
    {
        if (closed)
            return;
        if (wall_after <= cutoff_ns)
            lower_ns = cpu_elapsed;
        if (wall_before >= cutoff_ns) {
            upper_ns = cpu_elapsed;
            closed = true;
        }
    }

    void finish(uint64_t total_ns)
    {
        if (!closed)
            upper_ns = total_ns;
    }
};

struct WindowStageStats {
    uint64_t started{0};
    uint64_t completed{0};
    uint64_t late{0};
    uint64_t stale_seen{0};
    uint64_t dropped_stale{0};
    uint64_t cpu_ns{0};          /* Lower bound; includes in-flight compute. */
    uint64_t cpu_uncertainty_ns{0};
};

struct WindowQueueStats {
    uint64_t offered{0};        /* Actually delivered to this queue before T. */
    uint64_t dequeued{0};
    uint64_t evicted{0};

    uint64_t pending() const { return offered - dequeued - evicted; }
};

/* SPDX-License-Identifier: GPL-2.0 */
#include "../demo/window_metrics.h"
#include <cassert>
#include <cstdio>

int main()
{
    CpuWindowBounds before{100};
    before.sample(0, 10, 12);
    before.sample(20, 30, 32);
    before.finish(20);
    assert(before.lower_ns == 20 && before.upper_ns == 20);

    CpuWindowBounds after{100};
    after.sample(0, 110, 112);
    after.sample(100, 210, 212);
    after.finish(100);
    assert(after.lower_ns == 0 && after.upper_ns == 0);

    CpuWindowBounds crossing{100};
    crossing.sample(10, 90, 95);
    crossing.sample(20, 99, 110);  // Cannot locate this CPU sample relative to T.
    assert(crossing.lower_ns == 10 && !crossing.closed);
    crossing.sample(25, 120, 125);
    crossing.sample(1000, 2000, 2005);  // Drain cannot inflate window CPU.
    crossing.finish(1000);
    assert(crossing.lower_ns == 10 && crossing.upper_ns == 25);

    CpuWindowBounds ends_crossing{100};
    ends_crossing.sample(10, 90, 95);
    ends_crossing.sample(20, 99, 110);
    ends_crossing.finish(20);
    assert(ends_crossing.lower_ns == 10 && ends_crossing.upper_ns == 20);

    CpuWindowBounds preempted{100};
    preempted.sample(10, 90, 95);
    preempted.sample(12, 100000, 100005);  // Huge wall gap, only two CPU ns.
    preempted.finish(50);
    assert(preempted.lower_ns == 10 && preempted.upper_ns == 12);

    CpuWindowBounds exact{100};
    exact.sample(50, 100, 100);
    exact.finish(100);
    assert(exact.lower_ns == 50 && exact.upper_ns == 50);

    WindowQueueStats queue{10, 3, 2};
    assert(queue.pending() == 5);
    std::puts("Window CPU bounds and queue conservation passed");
}

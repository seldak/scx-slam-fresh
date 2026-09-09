// SPDX-License-Identifier: MIT
#include "../demo/dependent_graph.h"
#include <cassert>
#include <iostream>
using namespace dependent_graph;

static void conserved(const Graph &g) {
    for (auto s:{Stream::imu,Stream::camera}) {
        auto c=g.counts(s);
        assert(c.offered==c.dropped+c.consumed+g.pending(s)+g.in_flight(s));
    }
}
int main() {
    Graph g(2,2);
    assert(g.control(0,42,10).outcome==Outcome::missing);
    assert(g.offer({Stream::imu,1,1},1));
    assert(g.offer({Stream::camera,1,2},2));
    auto b=g.select(2); assert(b && b->inputs.size()==2);
    assert(b->inputs[0].stream==Stream::imu);
    assert(g.offer({Stream::imu,2,3},3));
    assert(!g.select(3)); // No overwrite of an executing batch.
    assert(b->inputs.size()==2 && b->inputs[0].sequence==1);
    assert(g.control(3,43,10).outcome==Outcome::missing);
    bool wrong=false;
    try { g.complete(b->id+1,3); } catch (const std::invalid_argument &) { wrong=true; }
    assert(wrong); conserved(g);
    auto first=g.complete(b->id,4);
    assert(first.imu->sequence==1 && first.camera->sequence==1);
    auto next=g.select(4); assert(next && next->id!=b->id);
    auto tick=g.control(5,44,10);
    assert(tick.outcome==Outcome::usable && tick.setpoint==44);
    assert(tick.snapshot->batch_id==first.batch_id); // Running work is invisible.
    assert(g.offer({Stream::imu,3,5},5));
    assert(g.offer({Stream::imu,4,6},6));
    assert(!g.offer({Stream::imu,5,7},7)); // Explicit reject-new overflow.
    conserved(g);
    g.complete(next->id,8); conserved(g);
    auto burst=g.select(8); assert(burst->inputs.size()==2);
    g.complete(burst->id,9); conserved(g);
    assert(g.control(20,45,10).outcome==Outcome::stale);
    // A fresh completion timestamp cannot make old measurements fresh.
    assert(g.offer({Stream::imu,6,4},21));
    auto old=g.select(21); g.complete(old->id,22);
    assert(g.control(22,46,10).outcome==Outcome::stale);
    conserved(g);
    assert(!g.select(22));
    Graph camera_only(1,1);
    camera_only.offer({Stream::camera,1,0},0);
    auto camera=camera_only.select(0); camera_only.complete(camera->id,1);
    assert(camera_only.control(1,1,100).outcome==Outcome::missing);
    std::cout<<"dependent graph ownership, batching, overflow, lineage and control tests passed\n";
}

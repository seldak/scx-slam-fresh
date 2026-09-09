// SPDX-License-Identifier: MIT
#pragma once
#include <algorithm>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <vector>

namespace dependent_graph {
using Time = uint64_t;
enum class Stream { imu, camera };
struct Measurement {
    Stream stream;
    uint64_t sequence;
    Time source_time;
};
struct Counts {
    uint64_t offered=0, dropped=0, consumed=0;
};
struct Batch {
    uint64_t id;
    std::vector<Measurement> inputs;
};
struct Snapshot {
    uint64_t batch_id;
    Time completed;
    std::optional<Measurement> imu, camera;
};
enum class Outcome { usable, stale, missing };
struct Control {
    uint64_t tick, setpoint;
    Time release;
    Outcome outcome;
    std::optional<Snapshot> snapshot;
};

// Caller serializes access. All times use one caller-supplied logical clock.
// Full inboxes reject the new input; running batches are never modified.
class Graph {
    size_t capacity, limit;
    std::deque<Measurement> inbox[2];
    Counts accounting[2];
    std::optional<uint64_t> last_sequence[2];
    std::optional<Batch> running;
    std::optional<Snapshot> latest;
    uint64_t next_batch=1, next_tick=1;
    Time clock_time=0;
    void advance(Time t) {
        if (t<clock_time) throw std::invalid_argument("clock moved backwards");
        clock_time=t;
    }
    static size_t index(Stream s) {
        if (s!=Stream::imu && s!=Stream::camera)
            throw std::invalid_argument("unknown stream");
        return s==Stream::imu?0:1;
    }
public:
    Graph(size_t capacity, size_t batch_limit):capacity(capacity),limit(batch_limit) {
        if (!capacity || !batch_limit) throw std::invalid_argument("zero queue/batch limit");
    }
    bool offer(Measurement m, Time arrival) {
        size_t i=index(m.stream);
        if (m.source_time>arrival ||
            (last_sequence[i] && m.sequence<=*last_sequence[i]))
            throw std::invalid_argument("invalid measurement identity/time");
        advance(arrival);
        last_sequence[i]=m.sequence;
        ++accounting[i].offered;
        if (inbox[i].size()==capacity) { ++accounting[i].dropped; return false; }
        inbox[i].push_back(m);
        return true;
    }
    std::optional<Batch> select(Time t) {
        advance(t);
        if (running) return std::nullopt;
        Batch b{next_batch,{}};
        while (b.inputs.size()<limit && (!inbox[0].empty() || !inbox[1].empty())) {
            size_t i=inbox[0].empty()?1:0;
            if (!inbox[0].empty() && !inbox[1].empty() &&
                inbox[1].front().source_time<inbox[0].front().source_time) i=1;
            b.inputs.push_back(inbox[i].front()); inbox[i].pop_front();
        }
        if (b.inputs.empty()) return std::nullopt;
        ++next_batch; running=b;
        return b;
    }
    Snapshot complete(uint64_t id, Time t) {
        if (!running || running->id!=id) throw std::invalid_argument("wrong completion owner");
        advance(t);
        Snapshot s=latest.value_or(Snapshot{});
        s.batch_id=id; s.completed=t;
        for (const auto &m:running->inputs) {
            ++accounting[index(m.stream)].consumed;
            auto &last=m.stream==Stream::imu?s.imu:s.camera;
            if (!last || m.source_time>=last->source_time) last=m;
        }
        latest=s; running.reset(); return s;
    }
    Control control(Time t, uint64_t setpoint, Time max_imu_age) {
        advance(t);
        Outcome outcome=Outcome::missing;
        if (latest && latest->imu)
            outcome=t-latest->imu->source_time<=max_imu_age?Outcome::usable:Outcome::stale;
        return {next_tick++,setpoint,t,outcome,latest};
    }
    Counts counts(Stream s) const { return accounting[index(s)]; }
    size_t pending(Stream s) const { return inbox[index(s)].size(); }
    size_t in_flight(Stream s) const {
        if (!running) return 0;
        return std::count_if(running->inputs.begin(),running->inputs.end(),
                            [s](const auto &m){return m.stream==s;});
    }
};
} // namespace dependent_graph

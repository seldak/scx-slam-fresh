// SPDX-License-Identifier: MIT
#include "dependent_graph.h"
#include "freshqos.h"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sched.h>
#include <string>
#include <thread>

using namespace dependent_graph;
constexpr Time ms=1000000;
static Time clock_ns(clockid_t id=CLOCK_MONOTONIC) {
    timespec t{};
    if(clock_gettime(id,&t)) throw std::runtime_error("clock_gettime failed");
    return Time(t.tv_sec)*1000000000+t.tv_nsec;
}
static void pin_cpu(int cpu) {
    cpu_set_t mask; CPU_ZERO(&mask); CPU_SET(cpu,&mask);
    if(pthread_setaffinity_np(pthread_self(),sizeof(mask),&mask))
        throw std::runtime_error("CPU affinity failed");
}
struct Job {
    uint64_t id=0;
    Time release=0, deadline=0, work=0;
    std::function<void(Time)> complete;
    Time assigned=0;
};
struct Result { Job job; Time start, end, cpu; };

// The dispatcher owns queued work. Each worker receives one published job.
class Worker {
    std::mutex mutex;
    std::condition_variable cv;
    std::thread thread;
    std::optional<Job> slot;
    std::optional<Result> result;
    bool stopping=false, ready=false;
    uint64_t tid=0;
    std::exception_ptr error;
    freshqos *qos;
    unsigned cls, stage;
    void run(int cpu) {
        try {
            pin_cpu(cpu);
            std::unique_lock<std::mutex> lock(mutex);
            tid=freshqos_pid_tgid_self();
            bool enrolled=false;
            if(qos) {
                enrollment_begin=clock_ns();
                if(freshqos_clear_hint(qos)) throw std::runtime_error("initial hint clear failed");
                sched_param p{};
                if(sched_setscheduler(0,7,&p)) throw std::runtime_error("SCHED_EXT enrollment failed");
                enrolled=true;
                enrollment_end=clock_ns();
            }
            ready=true; cv.notify_all();
            while(true) {
                cv.wait(lock,[&]{return stopping || slot.has_value();});
                if(stopping) break;
                Job job=std::move(*slot); slot.reset();
                lock.unlock();
                Time start=clock_ns(), before=clock_ns(CLOCK_THREAD_CPUTIME_ID);
                while(clock_ns(CLOCK_THREAD_CPUTIME_ID)-before<job.work) {}
                Time used=clock_ns(CLOCK_THREAD_CPUTIME_ID)-before, end=clock_ns();
                lock.lock();
                result=Result{std::move(job),start,end,used};
            }
            lock.unlock();
            shutdown_begin=clock_ns();
            if(enrolled) {
                sched_param p{};
                if(sched_setscheduler(0,SCHED_OTHER,&p)) throw std::runtime_error("scheduler restore failed");
            }
            if(qos && freshqos_clear_hint(qos)) throw std::runtime_error("hint clear failed");
            shutdown_end=clock_ns();
        } catch(...) {
            std::lock_guard<std::mutex> lock(mutex);
            error=std::current_exception(); ready=true; cv.notify_all();
        }
    }
public:
    std::string name;
    uint64_t offered=0,dropped=0,completed=0,late=0;
    Time cpu_time=0,max_start=0,max_age=0;
    Time enrollment_begin=0,enrollment_end=0,shutdown_begin=0,shutdown_end=0;
    struct Trace { uint64_t id; Time release, assigned, start, end, observed, deadline, cpu; };
    std::vector<Trace> traces;
    bool tracing=false;
    std::deque<Job> queue;
    bool busy=false;
    Worker(std::string name,unsigned cls,unsigned stage,int cpu,freshqos *qos):
        qos(qos),cls(cls),stage(stage),name(std::move(name)) {
        thread=std::thread([this,cpu]{run(cpu);});
        std::unique_lock<std::mutex> lock(mutex);
        cv.wait(lock,[&]{return ready;});
        if(error) {
            lock.unlock(); thread.join(); std::rethrow_exception(error);
        }
    }
    ~Worker() {
        finish();
    }
    void finish() {
        if(!thread.joinable()) return;
        {
            std::lock_guard<std::mutex> lock(mutex);
            // Only the normal drained path may retire the completed hint before
            // waking the worker. Never replace a still-owned job on error unwind.
            if(qos && !busy && !slot && !result && !error &&
               freshqos_publish_job_for(qos,tid,stage,FRESH_CLASS_BACKGROUND,0,0,0,0,0,0,0))
                error=std::make_exception_ptr(std::runtime_error("shutdown hint retirement failed"));
            stopping=true; cv.notify_all();
        }
        thread.join();
    }
    void check() const { if(error) std::rethrow_exception(error); }
    uint64_t identity() const { return tid; }
    unsigned service_class() const { return cls; }
    bool offer(Job job) {
        ++offered;
        if(queue.size()==32) {++dropped; return false;}
        queue.push_back(std::move(job)); return true;
    }
    void dispatch() {
        std::lock_guard<std::mutex> lock(mutex);
        if(error) std::rethrow_exception(error);
        if(busy || queue.empty()) return;
        const Job &j=queue.front();
        if(qos && freshqos_publish_job_for(qos,tid,stage,cls,j.id,j.release,
                j.deadline,0,0,0,0)) throw std::runtime_error("hint publication failed");
        slot=std::move(queue.front()); slot->assigned=clock_ns();
        queue.pop_front(); busy=true; cv.notify_one();
    }
    void collect() {
        std::optional<Result> done;
        {
            std::lock_guard<std::mutex> lock(mutex);
            if(error) std::rethrow_exception(error);
            done=std::move(result); result.reset();
        }
        if(!done) return;
        if(tracing) traces.push_back({done->job.id,done->job.release,done->job.assigned,
            done->start,done->end,clock_ns(),done->job.deadline,done->cpu});
        busy=false; ++completed; cpu_time+=done->cpu;
        max_start=std::max(max_start,done->start-done->job.release);
        max_age=std::max(max_age,done->end-done->job.release);
        late+=done->end>done->job.deadline;
        done->job.complete(done->end);
    }
};

int main(int argc,char **argv) {
    try {
        int cpu=-1, housekeeping=-1, seconds=2, hog_count=0;
        std::string pin, trace_path;
        for(int i=1;i<argc;i++) {
            std::string a=argv[i];
            if(a=="--help") {
                std::cout<<"--cpu N --housekeeping-cpu N [--duration SECONDS] [--hogs 0..2] [--pin BPF_DIRECTORY] [--trace FILE]\n";
                return 0;
            }
            if(i+1==argc) throw std::invalid_argument("missing option value");
            std::string v=argv[++i];
            if(a=="--pin") pin=v;
            else if(a=="--trace") trace_path=v;
            else {
                size_t used; int n=std::stoi(v,&used);
                if(used!=v.size()) throw std::invalid_argument("invalid integer");
                if(a=="--cpu") cpu=n;
                else if(a=="--housekeeping-cpu") housekeeping=n;
                else if(a=="--duration") seconds=n;
                else if(a=="--hogs") hog_count=n;
                else throw std::invalid_argument("unknown option");
            }
        }
        if(cpu<0 || housekeeping<0 || cpu>=CPU_SETSIZE || housekeeping>=CPU_SETSIZE ||
           cpu==housekeeping || seconds<1 || seconds>60 || hog_count<0 || hog_count>2)
            throw std::invalid_argument("require distinct CPUs and duration 1..60 seconds");
        pin_cpu(housekeeping);
        std::ofstream trace;
        if(!trace_path.empty()) {
            trace.open(trace_path);
            if(!trace) throw std::runtime_error("cannot open trace output");
        }
        freshqos q{.map_fd=-1};
        if(!pin.empty() && freshqos_open(&q,pin.c_str())) throw std::runtime_error("open hints failed");
        Graph graph(32,8);
        freshqos *hints=pin.empty()?nullptr:&q;
        // Deliberate synthetic profile, not measured robotics execution costs.
        Worker imu("imu_processing",FRESH_CLASS_DEADLINE,0,cpu,hints);
        Worker camera("camera_processing",FRESH_CLASS_DEADLINE,1,cpu,hints);
        Worker estimator("estimator",FRESH_CLASS_DEADLINE,2,cpu,hints);
        Worker control("control",FRESH_CLASS_URGENT,3,cpu,hints);
        Worker mapping("mapping",FRESH_CLASS_BACKGROUND,4,cpu,hints);
        std::vector<Worker *> workers{&imu,&camera,&estimator,&control,&mapping};
        std::vector<std::unique_ptr<Worker>> hogs;
        for(int i=0;i<hog_count;i++) {
            hogs.push_back(std::make_unique<Worker>("hog"+std::to_string(i),
                FRESH_CLASS_BACKGROUND,5+i,cpu,hints));
            workers.push_back(hogs.back().get());
        }
        for(auto *w:workers) w->tracing=!trace_path.empty();
        Time begin=clock_ns()+100*ms, end=begin+Time(seconds)*1000*ms;
        Time next_imu=begin,next_camera=begin,next_control=begin;
        uint64_t imu_id=0,camera_id=0,control_id=0,usable=0,stale=0,missing=0;
        std::vector<Time> control_ages;
        std::cout<<"profile=dependent-v1 synthetic=1 duration_s="<<seconds
                 <<" hints="<<!pin.empty()<<" worker_cpu="<<cpu<<" housekeeping_cpu="<<housekeeping
                 <<" hogs="<<hog_count
                 <<" queue_capacity=32 batch_limit=8 imu_period_us=5000 camera_period_us=50000"
                 <<" control_period_us=10000 max_imu_age_us=20000\n";
        while(true) {
            for(auto *w:workers) w->collect();
            Time t=clock_ns();
            if(t>=begin && t<end) for(auto &hog:hogs)
                if(!hog->busy && hog->queue.empty())
                    hog->offer({hog->offered+1,t,t+100*ms,ms,[](Time){}});
            while(next_imu<end && next_imu<=t) {
                Measurement m{Stream::imu,++imu_id,next_imu};
                imu.offer({m.sequence,next_imu,next_imu+5*ms,150000,[&,m](Time){graph.offer(m,clock_ns());}});
                next_imu+=5*ms;
            }
            while(next_camera<end && next_camera<=t) {
                Measurement m{Stream::camera,++camera_id,next_camera};
                camera.offer({m.sequence,next_camera,next_camera+33*ms,4*ms,[&,m](Time){graph.offer(m,clock_ns());}});
                next_camera+=50*ms;
            }
            while(next_control<end && next_control<=t) {
                Time release=next_control;
                uint64_t id=++control_id;
                if(control.busy || !control.queue.empty()) {
                    ++control.offered; ++control.dropped; next_control+=10*ms;
                    continue;
                }
                // Freeze the snapshot when selecting work, not when compute finishes.
                auto selection=graph.control(clock_ns(),id/100,20*ms);
                bool accepted=control.offer({id,release,release+10*ms,200000,[&,selection](Time){
                    usable+=selection.outcome==Outcome::usable;
                    stale+=selection.outcome==Outcome::stale;
                    missing+=selection.outcome==Outcome::missing;
                    if(selection.snapshot && selection.snapshot->imu)
                        control_ages.push_back(selection.release-selection.snapshot->imu->source_time);
                }});
                (void)accepted;
                next_control+=10*ms;
            }
            if(!estimator.busy && estimator.queue.empty()) {
                auto batch=graph.select(clock_ns());
                if(batch) {
                    Time release=batch->inputs.front().source_time;
                    Time cost=300000;
                    for(const auto &m:batch->inputs) cost+=m.stream==Stream::imu?100000:2*ms;
                    estimator.offer({batch->id,release,release+33*ms,cost,[&,batch](Time){
                        Snapshot snapshot=graph.complete(batch->id,clock_ns());
                        if(std::any_of(batch->inputs.begin(),batch->inputs.end(),[](const auto &m){return m.stream==Stream::camera;}))
                            mapping.offer({snapshot.batch_id,snapshot.camera->source_time,
                                snapshot.camera->source_time+100*ms,2*ms,
                                [snapshot](Time){(void)snapshot;}});
                    }});
                }
            }
            for(auto *w:workers) w->dispatch();
            bool pending=false;
            for(auto *w:workers) pending|=w->busy || !w->queue.empty();
            if(t>=end && !pending && !graph.pending(Stream::imu) && !graph.pending(Stream::camera)) break;
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        Time drained=clock_ns();
        for(auto *w:workers) w->finish();
        for(auto *w:workers) w->check();
        if(!trace_path.empty()) {
            trace<<"worker,job,release_ns,assigned_ns,start_ns,end_ns,observed_ns,deadline_ns,cpu_ns\n";
            for(auto *w:workers) for(const auto &r:w->traces)
                trace<<w->name<<','<<r.id<<','<<r.release<<','<<r.assigned<<','<<r.start<<','
                     <<r.end<<','<<r.observed<<','<<r.deadline<<','<<r.cpu<<'\n';
            trace.flush();
            if(!trace) throw std::runtime_error("trace write failed");
        }
        std::cout<<"measurement_window_ns="<<end-begin<<" begin_ns="<<begin<<" end_ns="<<end
                 <<" drain_ns="<<drained-end<<"\n";
        for(auto *w:workers)
            std::cout<<"lifecycle worker="<<w->name<<" enrollment_begin_ns="<<w->enrollment_begin
                     <<" tid="<<(w->identity() & 0xffffffffULL)<<" class="<<w->service_class()
                     <<" enrollment_end_ns="<<w->enrollment_end<<" shutdown_begin_ns="<<w->shutdown_begin
                     <<" shutdown_end_ns="<<w->shutdown_end<<"\n";
        for(auto *w:workers)
            std::cout<<"worker="<<w->name<<" offered="<<w->offered<<" completed="<<w->completed
                     <<" dropped="<<w->dropped<<" late="<<w->late<<" pending="<<w->queue.size()
                     <<" in_flight="<<w->busy<<" cpu_us="<<w->cpu_time/1000
                     <<" max_start_us="<<w->max_start/1000<<" max_age_us="<<w->max_age/1000<<"\n";
        for(auto s:{Stream::imu,Stream::camera}) {
            auto c=graph.counts(s);
            std::cout<<"stream="<<(s==Stream::imu?"imu":"camera")<<" offered="<<c.offered
                     <<" consumed="<<c.consumed<<" dropped="<<c.dropped<<" pending="<<graph.pending(s)
                     <<" in_flight="<<graph.in_flight(s)<<"\n";
        }
        std::cout<<"control usable="<<usable<<" stale="<<stale<<" missing="<<missing<<"\n";
        std::sort(control_ages.begin(),control_ages.end());
        std::cout<<"control_age samples="<<control_ages.size()
                 <<" p99_us="<<(control_ages.empty()?0:control_ages[(control_ages.size()*99+99)/100-1]/1000)
                 <<" max_us="<<(control_ages.empty()?0:control_ages.back()/1000)<<"\n";
        // q remains open until worker destructors clear their hints; process exit closes it.
        return 0;
    } catch(const std::exception &e) {std::cerr<<e.what()<<"\n"; return 1;}
}

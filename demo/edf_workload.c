/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>
#include "freshqos.h"

/* Independent implicit-deadline jobs: 84.14% utilization, two hyperperiods. */
static const uint64_t period[] = {50000000, 70000000};
static const uint64_t work[] = {21000000, 29500000};
static const uint64_t reservation[] = {21200000, 29700000};
static const unsigned count[] = {14, 10};
static uint64_t epoch;
static unsigned cpu;
static const char *mode;
static struct freshqos qos;
static pthread_barrier_t ready;
struct sample {
    uint64_t release, start, end, cpu;
};

static struct sample samples[2][14];

struct attributes {
    uint32_t size, policy;
    uint64_t flags;
    int32_t nice;
    uint32_t priority;
    uint64_t runtime, deadline, period;
};

static void fail(const char *what)
{
    perror(what);
    exit(2);
}

static uint64_t now(clockid_t clock)
{
    struct timespec t;

    if (clock_gettime(clock, &t))
        fail("clock_gettime");
    return (uint64_t)t.tv_sec * 1000000000ULL + t.tv_nsec;
}

static void sleep_until(uint64_t target)
{
    struct timespec t = {
        .tv_sec = target / 1000000000ULL,
        .tv_nsec = target % 1000000000ULL,
    };
    int err;

    do {
        err = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    } while (err == EINTR);

    if (err) {
        errno = err;
        fail("clock_nanosleep");
    }
}

static void publish(unsigned task, unsigned job)
{
    if (strcmp(mode, "scx"))
        return;

    uint64_t release = epoch + job * period[task];

    if (freshqos_publish_job(&qos, task, FRESH_CLASS_DEADLINE, job + 1, release,
                            release + period[task], 0, 0, 0, 0))
        fail("publish hint");
}

static void *worker(void *argument)
{
    unsigned task = *(unsigned *)argument;
    cpu_set_t mask;

    CPU_ZERO(&mask);
    CPU_SET(cpu, &mask);

    int err = pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask);

    if (err) {
        errno = err;
        fail("affinity");
    }

    struct sched_param param = {};
    int policy = SCHED_OTHER;

    publish(task, 0);
    if (!strcmp(mode, "deadline")) {
        struct attributes a = {
            .size = sizeof(a),
            .policy = 6,
            .runtime = reservation[task],
            .deadline = period[task],
            .period = period[task],
        };

        if (syscall(SYS_sched_setattr, 0, &a, 0)) {
            int saved = errno;

            fprintf(stderr, "task=%c runtime=%llu period=%llu cpu=%u\n", 'A' + task,
                    (unsigned long long)a.runtime, (unsigned long long)a.period, cpu);
            errno = saved;
            fail("SCHED_DEADLINE admission");
        }
        policy = 6;
    } else if (strcmp(mode, "ordinary")) {
        policy = !strcmp(mode, "scx") ? 7 : SCHED_FIFO;
        if (policy == SCHED_FIFO)
            param.sched_priority =
                ((!strcmp(mode, "fifo-a") && task == 0) ||
                 (!strcmp(mode, "fifo-b") && task == 1)) ? 60 : 50;
        if (sched_setscheduler(0, policy, &param))
            fail("enrollment");
    }

    struct sched_param actual = {};

    if (sched_getscheduler(0) != policy || sched_getparam(0, &actual) ||
        actual.sched_priority != param.sched_priority) {
        errno = EINVAL;
        fail("policy verification");
    }
    pthread_barrier_wait(&ready);
    if (now(CLOCK_MONOTONIC) >= epoch) {
        errno = ETIMEDOUT;
        fail("late enrollment");
    }

    for (unsigned j = 0; j < count[task]; j++) {
        if (j)
            publish(task, j);

        struct sample *s = &samples[task][j];

        s->release = epoch + j * period[task];
        sleep_until(s->release);
        s->start = now(CLOCK_MONOTONIC);

        uint64_t begin = now(CLOCK_THREAD_CPUTIME_ID);

        while (now(CLOCK_THREAD_CPUTIME_ID) - begin < work[task]) {
        }
        s->cpu = now(CLOCK_THREAD_CPUTIME_ID) - begin;
        s->end = now(CLOCK_MONOTONIC);
    }

    param.sched_priority = 0;
    if (sched_setscheduler(0, SCHED_OTHER, &param))
        fail("restore policy");
    if (!strcmp(mode, "scx") && freshqos_clear_hint(&qos))
        fail("clear hint");
    return NULL;
}

int main(int argc, char **argv)
{
    if (argc != 4) {
        fprintf(stderr, "usage: edf_workload CPU ordinary|fifo-a|fifo-b|deadline|scx PIN\n");
        return 2;
    }

    char *end;
    unsigned long parsed = strtoul(argv[1], &end, 10);

    if (*end || end == argv[1] || parsed >= CPU_SETSIZE)
        return 2;
    cpu = parsed;
    mode = argv[2];
    if (strcmp(mode, "ordinary") && strcmp(mode, "fifo-a") && strcmp(mode, "fifo-b") &&
        strcmp(mode, "deadline") && strcmp(mode, "scx"))
        return 2;
    if (!strcmp(mode, "scx") && freshqos_open(&qos, argv[3]))
        fail("open hints");

    epoch = now(CLOCK_MONOTONIC) + 500000000ULL;
    pthread_barrier_init(&ready, NULL, 2);

    pthread_t threads[2];
    unsigned ids[] = {0, 1};

    for (unsigned i = 0; i < 2; i++) {
        int err = pthread_create(&threads[i], NULL, worker, &ids[i]);

        if (err) {
            errno = err;
            fail("pthread_create");
        }
    }
    for (unsigned i = 0; i < 2; i++)
        pthread_join(threads[i], NULL);

    puts("task,job,release_ns,start_ns,end_ns,cpu_ns,deadline_ns");
    for (unsigned i = 0; i < 2; i++) {
        for (unsigned j = 0; j < count[i]; j++) {
            struct sample *s = &samples[i][j];

            printf("%c,%u,%llu,%llu,%llu,%llu,%llu\n", 'A' + i, j + 1,
                   (unsigned long long)s->release, (unsigned long long)s->start,
                   (unsigned long long)s->end, (unsigned long long)s->cpu,
                   (unsigned long long)(s->release + period[i]));
        }
    }

    if (!strcmp(mode, "scx"))
        freshqos_close(&qos);
    pthread_barrier_destroy(&ready);
    return 0;
}

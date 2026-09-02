#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
demo_bin="$repo_dir/build/slam_pipeline_demo"
loader_bin="$repo_dir/build/scx_slam_fresh_user"

cpu=${CPU:-0}
duration=${DURATION:-15}
sweep_duration=${SWEEP_DURATION:-8}
hog_threads=${HOG_THREADS:-2}
stale_hog_threads=${STALE_HOG_THREADS:-0}
burst_count=${BURST_COUNT:-12}
burst_at_ms=${BURST_AT_MS:-3000}
e3_hog_threads=${E3_HOG_THREADS:-2}
e3_normal_budget_us=${E3_NORMAL_BUDGET_US:-24000}
e3_low_budget_us=${E3_LOW_BUDGET_US:-1000}
e3_vision_work_us=${E3_VISION_WORK_US:-18000}
e3_vision_deadline_us=${E3_VISION_DEADLINE_US:-20000}
ext_policy=${EXT_POLICY:-7}
repetitions=${REPETITIONS:-1}
eval_scope=${EVAL_SCOPE:-all}
output_dir=${OUTPUT_DIR:-/tmp/scx-slam-fresh-eval-$(date +%Y%m%d-%H%M%S)-$$}
pin_dir="/sys/fs/bpf/scx_slam_fresh_eval_$$"
loader_pid=

usage() {
    cat <<EOF
Usage: sudo scripts/run_single_core_eval.sh

Optional environment variables:
  CPU=0 DURATION=15 SWEEP_DURATION=8 HOG_THREADS=2 STALE_HOG_THREADS=0 BURST_COUNT=12
  BURST_AT_MS=3000 E3_HOG_THREADS=2 E3_NORMAL_BUDGET_US=24000 E3_LOW_BUDGET_US=1000
  E3_VISION_WORK_US=18000
  E3_VISION_DEADLINE_US=20000
  EXT_POLICY=7 REPETITIONS=1 EVAL_SCOPE=all|e3
  OUTPUT_DIR=/path/to/results
EOF
}

cleanup() {
    if [[ -n "$loader_pid" ]] && kill -0 "$loader_pid" 2>/dev/null; then
        kill -TERM "$loader_pid" 2>/dev/null || true
        wait "$loader_pid" 2>/dev/null || true
    fi

    # Remove only the pins and directory created by this invocation.
    rm -f -- "$pin_dir/task_hints" "$pin_dir/events" || true
    rmdir -- "$pin_dir" 2>/dev/null || true
}

wait_for_scheduler() {
    for _ in {1..50}; do
        if [[ -e "$pin_dir/task_hints" ]] &&
           [[ -r /sys/kernel/sched_ext/state ]] &&
           [[ $(< /sys/kernel/sched_ext/state) == enabled ]]; then
            return 0
        fi

        if ! kill -0 "$loader_pid" 2>/dev/null; then
            echo "scheduler loader exited before attach" >&2
            tail -n 40 "$output_dir/loader.txt" >&2 || true
            return 1
        fi
        sleep 0.1
    done

    echo "timed out waiting for sched_ext attach" >&2
    return 1
}

run_case() {
    local name=$1
    local case_hogs=$2
    local lidar_mode=$3
    local case_duration=$4
    shift 4

    echo
    echo "=== $name ==="
    echo "Running for about ${case_duration}s; results are printed when the workload finishes."
    taskset -c "$cpu" "$demo_bin" \
        --lidar "$lidar_mode" \
        --hog "$case_hogs" \
        --duration "$case_duration" \
        "$@" 2>&1 | tee "$output_dir/$name.txt"
}

read_metric() {
    local stage=$1
    local metric=$2
    local result_file=$3

    awk -v stage="$stage:" -v metric="$metric" '
        $1 == stage {
            for (i = 2; i <= NF; i++) {
                if (index($i, metric "=") == 1) {
                    split($i, value, "=")
                    print value[2]
                    exit
                }
            }
        }
    ' "$result_file"
}

check_e0_sweep() {
    local repetition=$1
    local mode result_file generated processed pending stale reg_job_us
    local light_reg_job_us mid_reg_job_us heavy_reg_job_us

    for mode in light mid heavy; do
        result_file="$output_dir/cfs-sweep-$mode-$repetition.txt"
        generated=$(read_metric generated lidar "$result_file")
        processed=$(read_metric lidar_reg processed "$result_file")
        pending=$(read_metric lidar_reg pending "$result_file")
        stale=$(read_metric lidar_reg stale_seen "$result_file")
        reg_job_us=$(read_metric lidar_reg reg_job_us "$result_file")

        for metric in "$generated" "$processed" "$pending" "$stale" "$reg_job_us"; do
            if [[ ! $metric =~ ^[0-9]+$ ]]; then
                echo "error: could not parse E0 $mode sweep metrics" >&2
                return 1
            fi
        done

        case $mode in
            light)
                light_reg_job_us=$reg_job_us
                ;;
            mid)
                mid_reg_job_us=$reg_job_us
                ;;
            heavy)
                heavy_reg_job_us=$reg_job_us
                ;;
        esac

        if [[ $mode != heavy ]] &&
           (( processed != generated || pending != 0 || stale != 0 )); then
            echo "error: E0 $mode mode did not remain sustainable" >&2
            return 1
        fi

        if [[ $mode == heavy ]] &&
           (( processed >= generated || pending == 0 || stale == 0 )); then
            echo "error: E0 heavy mode did not create an unsustainable stale backlog" >&2
            return 1
        fi
    done

    if (( light_reg_job_us >= mid_reg_job_us ||
          mid_reg_job_us >= heavy_reg_job_us )); then
        echo "error: E0 LiDAR registration costs are not strictly increasing" >&2
        return 1
    fi

    if (( mid_reg_job_us < 50000 || mid_reg_job_us >= 100000 ||
          heavy_reg_job_us <= 100000 )); then
        echo "error: E0 mid/heavy registration costs no longer model borderline/overload regimes" >&2
        return 1
    fi
}

count_loader_events() {
    local start_marker=$1
    local end_marker=$2
    local event_kind=$3
    local stage_id=$4

    awk -v start="=== $start_marker ===" -v end="=== $end_marker ===" \
        -v event="$event_kind" -v stage="stage=$stage_id " '
        $0 == start { in_case = 1; next }
        $0 == end { in_case = 0 }
        in_case && index($0, "[evt] " event " ") && index($0, stage) { count++ }
        END { print count + 0 }
    ' "$output_dir/loader.txt"
}

check_e3_budget() {
    local repetition=$1
    local normal_file="$output_dir/scx-budget-normal-$repetition.txt"
    local low_file="$output_dir/scx-budget-low-$repetition.txt"
    local normal_late low_late normal_budget low_budget
    local normal_generated normal_processed low_processed normal_work low_work
    local normal_deadline low_deadline
    local normal_overruns normal_demotions low_overruns low_demotions

    normal_late=$(read_metric vision_fe late "$normal_file")
    low_late=$(read_metric vision_fe late "$low_file")
    normal_budget=$(read_metric configuration vision_budget_us "$normal_file")
    low_budget=$(read_metric configuration vision_budget_us "$low_file")
    normal_work=$(read_metric configuration vision_work_us "$normal_file")
    low_work=$(read_metric configuration vision_work_us "$low_file")
    normal_deadline=$(read_metric configuration vision_deadline_us "$normal_file")
    low_deadline=$(read_metric configuration vision_deadline_us "$low_file")
    normal_generated=$(read_metric generated camera "$normal_file")
    normal_processed=$(read_metric vision_fe processed "$normal_file")
    low_processed=$(read_metric vision_fe processed "$low_file")

    for metric in "$normal_late" "$low_late" "$normal_budget" "$low_budget" \
                  "$normal_work" "$low_work" "$normal_generated" \
                  "$normal_processed" "$low_processed" "$normal_deadline" \
                  "$low_deadline"; do
        if [[ ! $metric =~ ^[0-9]+$ ]]; then
            echo "error: could not parse E3 budget metrics" >&2
            return 1
        fi
    done

    normal_overruns=$(count_loader_events "scx-budget-normal-$repetition" \
        "scx-budget-low-$repetition" BUDGET_OVERRUN 1)
    normal_demotions=$(count_loader_events "scx-budget-normal-$repetition" \
        "scx-budget-low-$repetition" BUDGET_DEMOTION 1)
    low_overruns=$(count_loader_events "scx-budget-low-$repetition" \
        "scx-budget-end-$repetition" BUDGET_OVERRUN 1)
    low_demotions=$(count_loader_events "scx-budget-low-$repetition" \
        "scx-budget-end-$repetition" BUDGET_DEMOTION 1)

    if (( normal_budget != e3_normal_budget_us || low_budget != e3_low_budget_us )); then
        echo "error: E3 workload did not apply the requested vision budgets" >&2
        return 1
    fi

    if (( normal_work != e3_vision_work_us || low_work != e3_vision_work_us )); then
        echo "error: E3 workload did not apply the requested vision compute cost" >&2
        return 1
    fi

    if (( normal_deadline != e3_vision_deadline_us ||
          low_deadline != e3_vision_deadline_us )); then
        echo "error: E3 workload did not apply the requested vision deadline" >&2
        return 1
    fi

    if (( normal_generated == 0 || normal_processed != normal_generated ||
          low_processed == 0 )); then
        echo "error: E3 did not produce a healthy normal run and an active low-budget run" >&2
        return 1
    fi

    if (( normal_overruns != 0 || normal_demotions != 0 )); then
        echo "error: E3 normal vision budget unexpectedly caused an overrun or demotion" >&2
        return 1
    fi

    if (( low_overruns == 0 || low_demotions == 0 )); then
        echo "error: E3 low vision budget did not cause both overrun and demotion events" >&2
        return 1
    fi

    if (( low_late <= normal_late )); then
        echo "error: E3 low vision budget did not increase vision deadline misses" >&2
        return 1
    fi

    echo "E3 validated: vision late ${normal_late}->${low_late}; " \
         "low-budget overruns=$low_overruns demotions=$low_demotions"
}

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi

if (( $# != 0 )); then
    usage >&2
    exit 2
fi

if [[ $eval_scope != all && $eval_scope != e3 ]]; then
    echo "error: EVAL_SCOPE must be 'all' or 'e3'" >&2
    exit 2
fi

if (( EUID != 0 )); then
    echo "error: run this benchmark as root so it can attach sched_ext and access BPF maps" >&2
    echo "try: sudo scripts/run_single_core_eval.sh" >&2
    exit 1
fi

for binary in "$demo_bin" "$loader_bin"; do
    if [[ ! -x "$binary" ]]; then
        echo "error: missing $binary; run 'make' first" >&2
        exit 1
    fi
done

if [[ "$repo_dir/build/scx_slam_fresh.skel.h" -nt "$loader_bin" ]]; then
    echo "error: scheduler loader is older than its embedded BPF skeleton; run 'make' first" >&2
    exit 1
fi

if [[ "$repo_dir/demo/slam_pipeline_demo.cpp" -nt "$demo_bin" ]] ||
   [[ "$repo_dir/src/slamqos.c" -nt "$demo_bin" ]]; then
    echo "error: demo binary is older than its sources; run 'make' first" >&2
    exit 1
fi

if [[ ! -r /sys/kernel/sched_ext/state ]]; then
    echo "error: this kernel does not expose /sys/kernel/sched_ext/state" >&2
    exit 1
fi

if [[ $(< /sys/kernel/sched_ext/state) != disabled ]]; then
    echo "error: another sched_ext scheduler is already active" >&2
    exit 1
fi

mkdir -p -- "$output_dir"
trap cleanup EXIT INT TERM

if [[ $eval_scope == all ]]; then
    case_count=$((repetitions * 11))
    workload_seconds=$((repetitions * (8 * duration + 3 * sweep_duration)))
else
    case_count=$((repetitions * 2))
    workload_seconds=$((repetitions * 2 * duration))
fi
echo "Running $case_count cases (~$workload_seconds seconds of workload time)."
echo "Results directory: $output_dir"

cat >"$output_dir/environment.txt" <<EOF
date=$(date --iso-8601=seconds)
kernel=$(uname -r)
cpu=$cpu
duration=$duration
sweep_duration=$sweep_duration
hog_threads=$hog_threads
stale_hog_threads=$stale_hog_threads
burst_count=$burst_count
burst_at_ms=$burst_at_ms
e3_hog_threads=$e3_hog_threads
e3_normal_budget_us=$e3_normal_budget_us
e3_low_budget_us=$e3_low_budget_us
e3_vision_work_us=$e3_vision_work_us
e3_vision_deadline_us=$e3_vision_deadline_us
ext_policy=$ext_policy
repetitions=$repetitions
eval_scope=$eval_scope
git_commit=$(git -C "$repo_dir" rev-parse HEAD)
git_dirty=$(if [[ -n $(git -C "$repo_dir" status --porcelain) ]]; then echo yes; else echo no; fi)
demo_sha256=$(sha256sum "$demo_bin" | awk '{print $1}')
loader_sha256=$(sha256sum "$loader_bin" | awk '{print $1}')
bpf_object_sha256=$(sha256sum "$repo_dir/build/scx_slam_fresh.bpf.o" | awk '{print $1}')
EOF

if [[ $eval_scope == all ]]; then
    for repetition in $(seq 1 "$repetitions"); do
        for mode in light mid heavy; do
            run_case "cfs-sweep-$mode-$repetition" 0 "$mode" "$sweep_duration" --no-hints
        done
        check_e0_sweep "$repetition"

        run_case "cfs-isolation-$repetition" "$hog_threads" heavy "$duration" --no-hints
    done
fi

mkdir -p -- "$pin_dir"
stdbuf -oL -eL "$loader_bin" --pin "$pin_dir" >>"$output_dir/loader.txt" 2>&1 &
loader_pid=$!
wait_for_scheduler

for repetition in $(seq 1 "$repetitions"); do
    if [[ $eval_scope == all ]]; then
        if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
            echo "error: sched_ext stopped before scx run $repetition" >&2
            tail -n 40 "$output_dir/loader.txt" >&2 || true
            exit 1
        fi
        echo "=== scx-isolation-$repetition ===" >>"$output_dir/loader.txt"
        run_case "scx-isolation-$repetition" "$hog_threads" heavy "$duration" --pin "$pin_dir" --ext-policy "$ext_policy"

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before stale-keeping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-stale-keep-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-stale-keep-$repetition" "$stale_hog_threads" heavy "$duration" --pin "$pin_dir" --ext-policy "$ext_policy"

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before stale-dropping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-stale-drop-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-stale-drop-$repetition" "$stale_hog_threads" heavy "$duration" --pin "$pin_dir" --ext-policy "$ext_policy" --drop-stale 1

    keep_file="$output_dir/scx-stale-keep-$repetition.txt"
    drop_file="$output_dir/scx-stale-drop-$repetition.txt"
    keep_pending=$(read_metric lidar_reg pending "$keep_file")
    drop_pending=$(read_metric lidar_reg pending "$drop_file")
    drop_evicted=$(read_metric lidar_reg queue_evicted_stale "$drop_file")

    if [[ ! $keep_pending =~ ^[0-9]+$ ]] ||
       [[ ! $drop_pending =~ ^[0-9]+$ ]] ||
       [[ ! $drop_evicted =~ ^[0-9]+$ ]]; then
        echo "error: could not parse stale-shedding metrics" >&2
        exit 1
    fi

    if (( drop_evicted == 0 )); then
        echo "error: stale-dropping scenario did not evict queued LiDAR registration work" >&2
        exit 1
    fi

    if (( drop_pending >= keep_pending )); then
        echo "error: stale dropping did not reduce the pending LiDAR registration backlog" >&2
        exit 1
    fi

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before burst-keeping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-burst-keep-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-burst-keep-$repetition" 0 off "$duration" \
        --pin "$pin_dir" --ext-policy "$ext_policy" \
        --camera-burst-count "$burst_count" --camera-burst-at-ms "$burst_at_ms"

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before burst-dropping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-burst-drop-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-burst-drop-$repetition" 0 off "$duration" \
        --pin "$pin_dir" --ext-policy "$ext_policy" --drop-stale 1 \
        --camera-burst-count "$burst_count" --camera-burst-at-ms "$burst_at_ms"

    burst_keep_file="$output_dir/scx-burst-keep-$repetition.txt"
    burst_drop_file="$output_dir/scx-burst-drop-$repetition.txt"
    keep_burst_processed=$(read_metric state_est burst_processed "$burst_keep_file")
    drop_burst_processed=$(read_metric state_est burst_processed "$burst_drop_file")
    keep_burst_latest_seq=$(read_metric state_est burst_latest_seq "$burst_keep_file")
    drop_burst_latest_seq=$(read_metric state_est burst_latest_seq "$burst_drop_file")
    keep_burst_latest_age=$(read_metric state_est burst_latest_age_us "$burst_keep_file")
    drop_burst_latest_age=$(read_metric state_est burst_latest_age_us "$burst_drop_file")
    burst_dropped=$(read_metric vision_fe dropped_stale "$burst_drop_file")

    for metric in "$keep_burst_processed" "$drop_burst_processed" \
                  "$keep_burst_latest_seq" "$drop_burst_latest_seq" \
                  "$keep_burst_latest_age" "$drop_burst_latest_age" \
                  "$burst_dropped"; do
        if [[ ! $metric =~ ^[0-9]+$ ]]; then
            echo "error: could not parse sensor-burst metrics" >&2
            exit 1
        fi
    done

    if (( burst_dropped == 0 || drop_burst_processed >= keep_burst_processed )); then
        echo "error: burst stale dropping did not shed obsolete camera frames" >&2
        exit 1
    fi

    if (( drop_burst_latest_seq != keep_burst_latest_seq )); then
        echo "error: burst stale dropping failed to preserve the newest camera frame" >&2
        exit 1
    fi

        if (( drop_burst_latest_age >= keep_burst_latest_age )); then
            echo "error: burst stale dropping did not reduce newest-frame recovery age" >&2
            exit 1
        fi
    fi

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before normal-budget run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-budget-normal-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-budget-normal-$repetition" "$e3_hog_threads" off "$duration" \
        --pin "$pin_dir" --ext-policy "$ext_policy" \
        --vision-budget-us "$e3_normal_budget_us" \
        --vision-work-us "$e3_vision_work_us" \
        --vision-deadline-us "$e3_vision_deadline_us"
    sleep 0.2

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before low-budget run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-budget-low-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-budget-low-$repetition" "$e3_hog_threads" off "$duration" \
        --pin "$pin_dir" --ext-policy "$ext_policy" \
        --vision-budget-us "$e3_low_budget_us" \
        --vision-work-us "$e3_vision_work_us" \
        --vision-deadline-us "$e3_vision_deadline_us"
    sleep 0.2
    echo "=== scx-budget-end-$repetition ===" >>"$output_dir/loader.txt"
    check_e3_budget "$repetition"
done

echo
echo "Benchmark complete. Results: $output_dir"

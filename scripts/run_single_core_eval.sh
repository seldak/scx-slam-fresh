#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
demo_bin="$repo_dir/build/slam_pipeline_demo"
loader_bin="$repo_dir/build/scx_slam_fresh_user"

cpu=${CPU:-0}
duration=${DURATION:-15}
hog_threads=${HOG_THREADS:-2}
stale_hog_threads=${STALE_HOG_THREADS:-0}
burst_count=${BURST_COUNT:-12}
burst_at_ms=${BURST_AT_MS:-3000}
ext_policy=${EXT_POLICY:-7}
repetitions=${REPETITIONS:-1}
output_dir=${OUTPUT_DIR:-/tmp/scx-slam-fresh-eval-$(date +%Y%m%d-%H%M%S)-$$}
pin_dir="/sys/fs/bpf/scx_slam_fresh_eval_$$"
loader_pid=

usage() {
    cat <<EOF
Usage: sudo scripts/run_single_core_eval.sh

Optional environment variables:
  CPU=0 DURATION=15 HOG_THREADS=2 STALE_HOG_THREADS=0 BURST_COUNT=12
  BURST_AT_MS=3000 EXT_POLICY=7 REPETITIONS=1
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
    shift 3

    echo
    echo "=== $name ==="
    echo "Running for about ${duration}s; results are printed when the workload finishes."
    taskset -c "$cpu" "$demo_bin" \
        --lidar "$lidar_mode" \
        --hog "$case_hogs" \
        --duration "$duration" \
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

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi

if (( $# != 0 )); then
    usage >&2
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

echo "Running $((repetitions * 6)) cases (~$((repetitions * 6 * duration)) seconds of workload time)."
echo "Results directory: $output_dir"

cat >"$output_dir/environment.txt" <<EOF
date=$(date --iso-8601=seconds)
kernel=$(uname -r)
cpu=$cpu
duration=$duration
hog_threads=$hog_threads
stale_hog_threads=$stale_hog_threads
burst_count=$burst_count
burst_at_ms=$burst_at_ms
ext_policy=$ext_policy
repetitions=$repetitions
git_commit=$(git -C "$repo_dir" rev-parse HEAD)
git_dirty=$(if [[ -n $(git -C "$repo_dir" status --porcelain) ]]; then echo yes; else echo no; fi)
demo_sha256=$(sha256sum "$demo_bin" | awk '{print $1}')
loader_sha256=$(sha256sum "$loader_bin" | awk '{print $1}')
bpf_object_sha256=$(sha256sum "$repo_dir/build/scx_slam_fresh.bpf.o" | awk '{print $1}')
EOF

for repetition in $(seq 1 "$repetitions"); do
    run_case "cfs-isolation-$repetition" "$hog_threads" heavy --no-hints
done

mkdir -p -- "$pin_dir"
stdbuf -oL -eL "$loader_bin" --pin "$pin_dir" >>"$output_dir/loader.txt" 2>&1 &
loader_pid=$!
wait_for_scheduler

for repetition in $(seq 1 "$repetitions"); do
    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before scx run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-isolation-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-isolation-$repetition" "$hog_threads" heavy --pin "$pin_dir" --ext-policy "$ext_policy"

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before stale-keeping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-stale-keep-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-stale-keep-$repetition" "$stale_hog_threads" heavy --pin "$pin_dir" --ext-policy "$ext_policy"

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before stale-dropping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-stale-drop-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-stale-drop-$repetition" "$stale_hog_threads" heavy --pin "$pin_dir" --ext-policy "$ext_policy" --drop-stale 1

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
    run_case "scx-burst-keep-$repetition" 0 off \
        --pin "$pin_dir" --ext-policy "$ext_policy" \
        --camera-burst-count "$burst_count" --camera-burst-at-ms "$burst_at_ms"

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before burst-dropping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-burst-drop-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-burst-drop-$repetition" 0 off \
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
done

echo
echo "Benchmark complete. Results: $output_dir"

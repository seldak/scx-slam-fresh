#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
demo_bin="$repo_dir/build/slam_pipeline_demo"
loader_bin="$repo_dir/build/scx_slam_fresh_user"

cpu=${CPU:-0}
duration=${DURATION:-15}
hog_threads=${HOG_THREADS:-2}
ext_policy=${EXT_POLICY:-7}
repetitions=${REPETITIONS:-1}
output_dir=${OUTPUT_DIR:-/tmp/scx-slam-fresh-eval-$(date +%Y%m%d-%H%M%S)-$$}
pin_dir="/sys/fs/bpf/scx_slam_fresh_eval_$$"
loader_pid=

usage() {
    cat <<EOF
Usage: sudo scripts/run_single_core_eval.sh

Optional environment variables:
  CPU=0 DURATION=15 HOG_THREADS=2 EXT_POLICY=7 REPETITIONS=1
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
    shift

    echo
    echo "=== $name ==="
    echo "Running for about ${duration}s; results are printed when the workload finishes."
    taskset -c "$cpu" "$demo_bin" \
        --lidar heavy \
        --hog "$hog_threads" \
        --duration "$duration" \
        "$@" 2>&1 | tee "$output_dir/$name.txt"
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

echo "Running $((repetitions * 3)) cases (~$((repetitions * 3 * duration)) seconds of workload time)."
echo "Results directory: $output_dir"

cat >"$output_dir/environment.txt" <<EOF
date=$(date --iso-8601=seconds)
kernel=$(uname -r)
cpu=$cpu
duration=$duration
hog_threads=$hog_threads
ext_policy=$ext_policy
repetitions=$repetitions
git_commit=$(git -C "$repo_dir" rev-parse HEAD)
git_dirty=$(if [[ -n $(git -C "$repo_dir" status --porcelain) ]]; then echo yes; else echo no; fi)
demo_sha256=$(sha256sum "$demo_bin" | awk '{print $1}')
loader_sha256=$(sha256sum "$loader_bin" | awk '{print $1}')
bpf_object_sha256=$(sha256sum "$repo_dir/build/scx_slam_fresh.bpf.o" | awk '{print $1}')
EOF

for repetition in $(seq 1 "$repetitions"); do
    run_case "cfs-$repetition" --no-hints
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
    echo "=== scx-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-$repetition" --pin "$pin_dir" --ext-policy "$ext_policy"

    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before stale-dropping run $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi
    echo "=== scx-drop-stale-$repetition ===" >>"$output_dir/loader.txt"
    run_case "scx-drop-stale-$repetition" --pin "$pin_dir" --ext-policy "$ext_policy" --drop-stale 1
done

echo
echo "Benchmark complete. Results: $output_dir"

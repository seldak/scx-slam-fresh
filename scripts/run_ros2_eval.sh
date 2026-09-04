#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
loader_bin="$repo_dir/build/scx_slam_fresh_user"
ros_bin="$repo_dir/.ros2-install/scx_slam_workload/lib/scx_slam_workload/scx_slam_pipeline"
ros_setup=${ROS2_SETUP:-/opt/ros/lyrical/setup.bash}
local_setup="$repo_dir/.ros2-install/setup.bash"

cpu=${CPU:-0}
housekeeping_cpu=${HOUSEKEEPING_CPU:-1}
duration=${DURATION:-15}
hog_threads=${HOG_THREADS:-2}
ext_policy=${EXT_POLICY:-7}
repetitions=${REPETITIONS:-3}
scx_variants=${SCX_VARIANTS:-hinted}
output_dir=${OUTPUT_DIR:-/tmp/scx-slam-fresh-ros2-eval-$(date +%Y%m%d-%H%M%S)-$$}
pin_dir="/sys/fs/bpf/scx_slam_fresh_ros2_eval_$$"
loader_pid=

usage() {
    cat <<EOF
Usage: sudo scripts/run_ros2_eval.sh

Run 'make && make ros2 && make test-ros2' as your normal user first.

Optional environment variables:
  CPU=0 HOUSEKEEPING_CPU=1 DURATION=15 HOG_THREADS=2
  EXT_POLICY=7 REPETITIONS=3 SCX_VARIANTS=hinted OUTPUT_DIR=/path/to/results
  ROS2_SETUP=/opt/ros/lyrical/setup.bash

SCX_VARIANTS is a comma-separated subset of:
  hinted,no-hints,imu-only,fe-only
EOF
}

cleanup() {
    if [[ -n "$loader_pid" ]] && kill -0 "$loader_pid" 2>/dev/null; then
        kill -TERM "$loader_pid" 2>/dev/null || true
        wait "$loader_pid" 2>/dev/null || true
    fi
    rm -f -- "$pin_dir/task_hints" "$pin_dir/events" || true
    rmdir -- "$pin_dir" 2>/dev/null || true
}

wait_for_scheduler() {
    for _ in {1..50}; do
        if [[ -e "$pin_dir/task_hints" ]] &&
           [[ -r /sys/kernel/sched_ext/state ]] &&
           [[ $(< /sys/kernel/sched_ext/state) == enabled ]]; then
            if [[ $(< /sys/kernel/sched_ext/switch_all) != 0 ]]; then
                echo "error: scheduler attached in full-switch mode; partial mode is required" >&2
                return 1
            fi
            return 0
        fi
        if ! kill -0 "$loader_pid" 2>/dev/null; then
            echo "error: scheduler loader exited before attach" >&2
            tail -n 40 "$output_dir/loader.txt" >&2 || true
            return 1
        fi
        sleep 0.1
    done
    echo "error: timed out waiting for sched_ext attach" >&2
    return 1
}

append_summary() {
    local mode=$1
    local repetition=$2
    local result_file=$3

    awk -v mode="$mode" -v repetition="$repetition" '
        /^window_[a-z_]+:/ {
            stage = $1
            sub(/^window_/, "", stage)
            sub(/:$/, "", stage)
            delete value
            for (i = 2; i <= NF; i++) {
                split($i, pair, "=")
                value[pair[1]] = pair[2]
            }
            printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n", \
                mode, repetition, stage, value["offered"], value["completed"], \
                value["late"], value["started_stale"], value["unfinished"], value["cpu_us"], \
                value["p99_start_age_us"], value["max_start_age_us"], \
                value["p99_age_us"], value["max_age_us"]
        }
    ' "$result_file" >>"$output_dir/summary.tsv"
}

run_case() {
    local mode=$1
    local repetition=$2
    shift 2
    local name="${mode}-${repetition}"
    local result_file="$output_dir/$name.txt"

    echo
    echo "=== $name ==="
    taskset -c "$housekeeping_cpu" "$ros_bin" \
        --duration "$duration" \
        --worker-cpu "$cpu" \
        --hog "$hog_threads" \
        --window-stats \
        "$@" 2>&1 | tee "$result_file"
    append_summary "$mode" "$repetition" "$result_file"
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
    echo "error: run as root so the harness can attach sched_ext and set SCHED_EXT" >&2
    exit 1
fi
if (( cpu == housekeeping_cpu )); then
    echo "error: CPU and HOUSEKEEPING_CPU must differ" >&2
    exit 2
fi
for value in "$cpu" "$housekeeping_cpu" "$duration" "$hog_threads" "$ext_policy" "$repetitions"; do
    if [[ ! $value =~ ^[0-9]+$ ]]; then
        echo "error: numeric options must contain non-negative integers" >&2
        exit 2
    fi
done
IFS=',' read -r -a variant_list <<<"$scx_variants"
if (( ${#variant_list[@]} == 0 )); then
    echo "error: SCX_VARIANTS must select at least one variant" >&2
    exit 2
fi
for variant in "${variant_list[@]}"; do
    case "$variant" in
        hinted|no-hints|imu-only|fe-only) ;;
        *)
            echo "error: unknown SCX variant '$variant'" >&2
            exit 2
            ;;
    esac
done
if (( duration < 1 || repetitions < 1 )); then
    echo "error: DURATION and REPETITIONS must be positive" >&2
    exit 2
fi
for file in "$loader_bin" "$ros_bin"; do
    if [[ ! -x $file ]]; then
        echo "error: missing $file; run 'make && make ros2' as your normal user" >&2
        exit 1
    fi
done
for file in "$ros_setup" "$local_setup"; do
    if [[ ! -f $file ]]; then
        echo "error: missing ROS setup $file; run 'make ros2' as your normal user" >&2
        exit 1
    fi
done
if [[ ! -r /sys/kernel/sched_ext/state ]]; then
    echo "error: this kernel does not expose sched_ext" >&2
    exit 1
fi
if [[ $(< /sys/kernel/sched_ext/state) != disabled ]]; then
    echo "error: another sched_ext scheduler is already active" >&2
    exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ros_setup"
# shellcheck disable=SC1090
source "$local_setup"
set -u

ops_flags=$("$loader_bin" --print-ops-flags)
if [[ ! $ops_flags =~ ^0x[0-9a-fA-F]+$ ]] || (( (ops_flags & 8) == 0 )); then
    echo "error: embedded ops_flags=$ops_flags lacks SCX_OPS_SWITCH_PARTIAL" >&2
    exit 1
fi

mkdir -p -- "$output_dir" "$pin_dir"
trap cleanup EXIT INT TERM

cat >"$output_dir/environment.txt" <<EOF
date=$(date --iso-8601=seconds)
kernel=$(uname -r)
ros_distro=${ROS_DISTRO:-unknown}
rmw_implementation=${RMW_IMPLEMENTATION:-default}
cpu=$cpu
housekeeping_cpu=$housekeeping_cpu
duration=$duration
hog_threads=$hog_threads
ext_policy=$ext_policy
ops_flags=$ops_flags
repetitions=$repetitions
scx_variants=$scx_variants
git_commit=$(git -c safe.directory="$repo_dir" -C "$repo_dir" rev-parse HEAD)
git_dirty=$(if [[ -n $(git -c safe.directory="$repo_dir" -C "$repo_dir" status --porcelain) ]]; then echo yes; else echo no; fi)
ros_pipeline_sha256=$(sha256sum "$(readlink -f "$ros_bin")" | awk '{print $1}')
loader_sha256=$(sha256sum "$loader_bin" | awk '{print $1}')
bpf_object_sha256=$(sha256sum "$repo_dir/build/scx_slam_fresh.bpf.o" | awk '{print $1}')
EOF

printf "mode\trepetition\tstage\toffered\tcompleted\tlate\tstarted_stale\tunfinished\tcpu_us\tp99_start_age_us\tmax_start_age_us\tp99_age_us\tmax_age_us\n" >"$output_dir/summary.tsv"

echo "Results directory: $output_dir"
echo "Running matched CFS and scx_slam cases on CPU $cpu; DDS/dispatch starts on CPU $housekeeping_cpu."
for repetition in $(seq 1 "$repetitions"); do
    run_case cfs "$repetition"
done

taskset -c "$housekeeping_cpu" stdbuf -oL -eL "$loader_bin" --pin "$pin_dir" \
    >>"$output_dir/loader.txt" 2>&1 &
loader_pid=$!
wait_for_scheduler

for variant in "${variant_list[@]}"; do
    case "$variant" in
        hinted) hint_mode=full ;;
        *) hint_mode=$variant ;;
    esac
    for repetition in $(seq 1 "$repetitions"); do
        if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
            echo "error: sched_ext stopped before $variant case $repetition" >&2
            tail -n 40 "$output_dir/loader.txt" >&2 || true
            exit 1
        fi
        echo "=== ${variant}-${repetition} ===" >>"$output_dir/loader.txt"
        run_case "$variant" "$repetition" \
            --pin "$pin_dir" --ext-policy "$ext_policy" --hint-mode "$hint_mode"
    done
done

echo
echo "Phase 3 exploration complete; no comparative win is asserted."
echo "Results: $output_dir"
echo "Machine-readable metrics: $output_dir/summary.tsv"

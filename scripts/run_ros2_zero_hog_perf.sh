#!/usr/bin/env bash
# SPDX-License-Identifier: MIT

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
loader_bin="$repo_dir/build/scx_slam_fresh_user"
ros_bin="$repo_dir/.ros2-install/scx_slam_workload/lib/scx_slam_workload/scx_slam_pipeline"
ros_setup=${ROS2_SETUP:-/opt/ros/lyrical/setup.bash}
local_setup="$repo_dir/.ros2-install/setup.bash"

cpu=${CPU:-0}
housekeeping_cpu=${HOUSEKEEPING_CPU:-1}
duration=${DURATION:-15}
ext_policy=${EXT_POLICY:-7}
repetitions=${REPETITIONS:-3}
output_dir=${OUTPUT_DIR:-/tmp/scx-slam-fresh-ros2-perf-$(date +%Y%m%d-%H%M%S)-$$}
pin_dir="/sys/fs/bpf/scx_slam_fresh_ros2_perf_$$"
loader_pid=

# The zero-hog question was observed with this exact f01e3f9 binary set.
expected_ros_sha=38a15888215e17aced3a0c8ffe99932bd0c61adb2ca0f35c77d63ec5293920e5
expected_loader_sha=3dd5519925781964d451c1039017404169d8384deb835bce811e95ddca35243f
expected_bpf_sha=75719c3ac6959b41e80fdeb4975538aa28b9bb65eadfa42eedc6e858d42a1860

usage() {
    cat <<EOF
Usage: sudo scripts/run_ros2_zero_hog_perf.sh

Fixed diagnostic: hinted SCX, zero hogs, workers on CPU 0, ROS dispatch on CPU 1.

Optional environment variables:
  CPU=0 HOUSEKEEPING_CPU=1 DURATION=15 EXT_POLICY=7 REPETITIONS=3
  OUTPUT_DIR=/path/to/results ROS2_SETUP=/opt/ros/lyrical/setup.bash
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
                echo "error: scheduler attached in full-switch mode" >&2
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

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi
if (( $# != 0 )); then
    usage >&2
    exit 2
fi
if (( EUID != 0 )); then
    echo "error: run as root so perf and sched_ext can attach" >&2
    exit 1
fi
if (( cpu == housekeeping_cpu )); then
    echo "error: CPU and HOUSEKEEPING_CPU must differ" >&2
    exit 2
fi
for value in "$cpu" "$housekeeping_cpu" "$duration" "$ext_policy" "$repetitions"; do
    if [[ ! $value =~ ^[0-9]+$ ]]; then
        echo "error: numeric options must contain non-negative integers" >&2
        exit 2
    fi
done
if (( duration < 1 || repetitions < 1 )); then
    echo "error: DURATION and REPETITIONS must be positive" >&2
    exit 2
fi
for command in perf taskset stdbuf; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "error: missing required command: $command" >&2
        exit 1
    fi
done
for file in "$loader_bin" "$ros_bin"; do
    if [[ ! -x $file ]]; then
        echo "error: missing $file; run 'make && make ros2' as your normal user" >&2
        exit 1
    fi
done
ros_sha=$(sha256sum "$(readlink -f "$ros_bin")" | awk '{print $1}')
loader_sha=$(sha256sum "$loader_bin" | awk '{print $1}')
bpf_sha=$(sha256sum "$repo_dir/build/scx_slam_fresh.bpf.o" | awk '{print $1}')
if [[ $ros_sha != "$expected_ros_sha" ]] ||
   [[ $loader_sha != "$expected_loader_sha" ]] ||
   [[ $bpf_sha != "$expected_bpf_sha" ]]; then
    echo "error: binaries differ from the f01e3f9 zero-hog observation" >&2
    echo "ROS pipeline: $ros_sha" >&2
    echo "loader: $loader_sha" >&2
    echo "BPF object: $bpf_sha" >&2
    exit 1
fi
for file in "$ros_setup" "$local_setup"; do
    if [[ ! -f $file ]]; then
        echo "error: missing ROS setup $file; run 'make ros2' as your normal user" >&2
        exit 1
    fi
done
if [[ ! -r /sys/kernel/sched_ext/state ]] ||
   [[ $(< /sys/kernel/sched_ext/state) != disabled ]]; then
    echo "error: sched_ext is unavailable or another scheduler is active" >&2
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
cpu=$cpu
housekeeping_cpu=$housekeeping_cpu
duration=$duration
hog_threads=0
hint_mode=full
ext_policy=$ext_policy
ops_flags=$ops_flags
repetitions=$repetitions
binary_source_pin=f01e3f91c519325e5e06f0d42fd0a08bb872cc85
git_commit=$(git -c safe.directory="$repo_dir" -C "$repo_dir" rev-parse HEAD)
git_dirty=$(if [[ -n $(git -c safe.directory="$repo_dir" -C "$repo_dir" status --porcelain) ]]; then echo yes; else echo no; fi)
ros_pipeline_sha256=$ros_sha
loader_sha256=$loader_sha
bpf_object_sha256=$bpf_sha
perf_version=$(perf version)
perf_events=sched_switch,sched_wakeup,sched_wakeup_new,sched_process_fork,sched_migrate_task
incoming_scx_slice=unavailable_in_standard_sched_tracepoints
EOF

taskset -c "$housekeeping_cpu" stdbuf -oL -eL "$loader_bin" --pin "$pin_dir" \
    >>"$output_dir/loader.txt" 2>&1 &
loader_pid=$!
wait_for_scheduler

events=(
    -e sched:sched_switch
    -e sched:sched_wakeup
    -e sched:sched_wakeup_new
    -e sched:sched_process_fork
    -e sched:sched_migrate_task
)

echo "Results directory: $output_dir"
echo "Capturing hinted zero-hog SCX on CPUs $cpu,$housekeeping_cpu."
for repetition in $(seq 1 "$repetitions"); do
    if [[ $(< /sys/kernel/sched_ext/state) != enabled ]]; then
        echo "error: sched_ext stopped before repetition $repetition" >&2
        tail -n 40 "$output_dir/loader.txt" >&2 || true
        exit 1
    fi

    case_dir="$output_dir/rep-$repetition"
    mkdir -p -- "$case_dir"
    echo
    echo "=== perf-zero-hog-$repetition ==="
    echo "=== perf-zero-hog-$repetition ===" >>"$output_dir/loader.txt"

    taskset -c "$housekeeping_cpu" perf record -a \
        -C "$cpu,$housekeeping_cpu" -k mono -m 1024 -c 1 \
        "${events[@]}" -N -B --no-buildid-mmap \
        -o "$case_dir/perf.data" -- \
        taskset -c "$housekeeping_cpu" "$ros_bin" \
            --duration "$duration" \
            --worker-cpu "$cpu" \
            --hog 0 \
            --window-stats \
            --pin "$pin_dir" \
            --ext-policy "$ext_policy" \
            --hint-mode full \
        2>&1 | tee "$case_dir/workload.txt"

    perf script -i "$case_dir/perf.data" --ns >"$case_dir/perf-script.txt"
    perf sched timehist -i "$case_dir/perf.data" -C "$cpu,$housekeeping_cpu" \
        --wakeups --next --cpu-visual --state \
        >"$case_dir/timehist.txt"
done

echo
echo "Capture complete; no tail cause or policy fix is asserted."
echo "Results: $output_dir"
echo "Standard sched tracepoints do not expose incoming scx.slice; timehist reports observed run intervals."

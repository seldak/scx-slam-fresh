#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0

set -euo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
loader_bin="$repo_dir/build/scx_slam_fresh_user"
pipeline_bin="$repo_dir/.ros2-install/scx_slam_workload/lib/scx_slam_workload/scx_slam_pipeline"
adapter_bin="$repo_dir/.ros2-install/scx_slam_workload/lib/scx_slam_workload/scx_slam_bag_adapter"
ros_setup=${ROS2_SETUP:-/opt/ros/lyrical/setup.bash}
local_setup="$repo_dir/.ros2-install/setup.bash"

cpu=${CPU:-14}
housekeeping_cpu=${HOUSEKEEPING_CPU:-1}
duration=${DURATION:-15}
warmup=${WARMUP:-3}
bag_offset=${BAG_OFFSET:-0}
repetitions=${REPETITIONS:-3}
hog_threads=${HOG_THREADS:-0}
ext_policy=${EXT_POLICY:-7}
imu_topic=${IMU_TOPIC:-/imu0}
camera_topic=${CAMERA_TOPIC:-/cam0/image_raw}
output_dir=${OUTPUT_DIR:-/tmp/scx-slam-fresh-ros2-bag-$(date +%Y%m%d-%H%M%S)-$$}
pin_dir="/sys/fs/bpf/scx_slam_fresh_ros2_bag_$$"
loader_pid=
adapter_pid=
pipeline_pid=
player_pid=

usage() {
    cat <<EOF
Usage: sudo scripts/run_ros2_bag_eval.sh /absolute/path/to/ros2-bag

Run 'make && make ros2 && make test-ros2' as your normal user first.

Defaults:
  CPU=14 HOUSEKEEPING_CPU=1 DURATION=15 WARMUP=3 BAG_OFFSET=0
  REPETITIONS=3 HOG_THREADS=0 EXT_POLICY=7
  IMU_TOPIC=/imu0 CAMERA_TOPIC=/cam0/image_raw

The harness runs three CFS repetitions followed by three hinted partial-switch
SCX repetitions. Dataset content remains outside the repository.
EOF
}

cleanup_process() {
    local pid=${1:-}
    local signal=${2:-INT}
    if [[ -n $pid ]] && kill -0 "$pid" 2>/dev/null; then
        kill -"$signal" "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    fi
}

cleanup() {
    cleanup_process "$player_pid"
    cleanup_process "$pipeline_pid" TERM
    cleanup_process "$adapter_pid"
    cleanup_process "$loader_pid" TERM
    rm -f -- "$pin_dir/task_hints" "$pin_dir/events" 2>/dev/null || true
    rmdir -- "$pin_dir" 2>/dev/null || true
}

wait_for_scheduler() {
    for _ in {1..50}; do
        if [[ -e $pin_dir/task_hints ]] &&
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

wait_for_topic_endpoints() {
    local topic=$1
    local publishers=$2
    local subscriptions=$3
    local info
    for _ in {1..100}; do
        info=$(ros2 topic info "$topic" 2>/dev/null || true)
        if grep -q "Publisher count: $publishers" <<<"$info" &&
           grep -q "Subscription count: $subscriptions" <<<"$info"; then
            return 0
        fi
        sleep 0.1
    done
    echo "error: topic endpoints did not connect for $topic" >&2
    return 1
}

write_qos_overrides() {
    cat >"$output_dir/qos-overrides.yaml" <<EOF
$imu_topic:
  history: keep_last
  depth: 1000
  reliability: reliable
  durability: volatile
$camera_topic:
  history: keep_last
  depth: 1000
  reliability: reliable
  durability: volatile
EOF
}

start_adapter() {
    local log=$1
    taskset -c "$housekeeping_cpu" stdbuf -oL -eL "$adapter_bin" --ros-args \
        -p "imu_input:=$imu_topic" -p "camera_input:=$camera_topic" \
        >"$log" 2>&1 &
    adapter_pid=$!
    wait_for_topic_endpoints /imu/jobs 1 0
    wait_for_topic_endpoints /camera/jobs 1 0
}

start_player() {
    local log=$1
    local paused=${2:-0}
    local play_duration=$((warmup + duration + 2))
    local args=(
        bag play "$bag_path" --rate 1.0
        --start-offset "$bag_offset"
        --playback-duration "$play_duration"
        --topics "$imu_topic" "$camera_topic"
        --qos-profile-overrides-path "$output_dir/qos-overrides.yaml"
        --disable-keyboard-controls
        --progress-bar-update-rate 0
    )
    if (( paused )); then
        args+=(--start-paused)
    else
        args+=(--delay 1)
    fi
    taskset -c "$housekeeping_cpu" ros2 "${args[@]}" >"$log" 2>&1 &
    player_pid=$!
}

qos_preflight() {
    echo "Checking bag-player and adapter QoS endpoints..."
    start_adapter "$output_dir/qos-adapter.txt"
    start_player "$output_dir/qos-player.txt" 1
    wait_for_topic_endpoints "$imu_topic" 1 1
    wait_for_topic_endpoints "$camera_topic" 1 1
    ros2 topic info -v "$imu_topic" >"$output_dir/qos-imu.txt"
    ros2 topic info -v "$camera_topic" >"$output_dir/qos-camera.txt"
    if ! grep -q 'Topic type: sensor_msgs/msg/Imu' "$output_dir/qos-imu.txt" ||
       ! grep -q 'Topic type: sensor_msgs/msg/Image' "$output_dir/qos-camera.txt"; then
        echo "error: bag topics do not expose the required sensor message types" >&2
        return 1
    fi
    for file in "$output_dir/qos-imu.txt" "$output_dir/qos-camera.txt"; do
        if (( $(grep -c 'Reliability: RELIABLE' "$file") < 2 )); then
            echo "error: explicit reliable QoS did not match in $file" >&2
            return 1
        fi
    done
    cleanup_process "$player_pid"
    player_pid=
    cleanup_process "$adapter_pid"
    adapter_pid=
}

assert_case_accounting() {
    local pipeline_log=$1
    local adapter_log=$2
    local name=$3
    if ! grep -q 'adapter_imu: .*dropped=0' "$adapter_log" ||
       ! grep -q 'adapter_camera: .*dropped=0' "$adapter_log"; then
        echo "error: adapter dropped input in $name" >&2
        return 1
    fi
    awk '
        /^window_imu_prop:/ || /^window_vision_fe:/ {
            delete value
            for (i = 2; i <= NF; i++) {
                split($i, pair, "=")
                value[pair[1]] = pair[2]
            }
            span = value["last_job_id"] - value["first_job_id"] + 1
            if (value["offered"] == 0 || span != value["offered"]) exit 1
        }
    ' "$pipeline_log" || {
        echo "error: source job-id span does not match taken callbacks in $name" >&2
        return 1
    }
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
            printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n", \
                mode, repetition, stage, value["offered"], value["completed"], \
                value["late"], value["started_stale"], value["unfinished"], \
                value["cpu_us"], value["p99_start_age_us"], value["max_start_age_us"], \
                value["p99_age_us"], value["max_age_us"], value["first_source_ts_ns"], \
                value["last_source_ts_ns"]
        }
    ' "$result_file" >>"$output_dir/summary.tsv"
}

run_case() {
    local mode=$1
    local repetition=$2
    local name="${mode}-${repetition}"
    local case_dir="$output_dir/$name"
    local pipeline_args=(--duration "$duration" --warmup "$warmup" --input external
        --worker-cpu "$cpu" --hog "$hog_threads" --window-stats)
    if [[ $mode == hinted ]]; then
        pipeline_args+=(--pin "$pin_dir" --ext-policy "$ext_policy" --hint-mode full)
    fi

    mkdir -p "$case_dir"
    echo
    echo "=== $name ==="
    start_adapter "$case_dir/adapter.txt"
    taskset -c "$housekeeping_cpu" stdbuf -oL -eL "$pipeline_bin" \
        "${pipeline_args[@]}" >"$case_dir/pipeline.txt" 2>&1 &
    pipeline_pid=$!
    start_player "$case_dir/player.txt"
    wait "$pipeline_pid"
    pipeline_pid=
    if ! kill -0 "$player_pid" 2>/dev/null; then
        echo "error: bag playback ended before the measurement process in $name" >&2
        return 1
    fi
    cleanup_process "$player_pid"
    player_pid=
    cleanup_process "$adapter_pid"
    adapter_pid=
    cat "$case_dir/pipeline.txt"
    assert_case_accounting "$case_dir/pipeline.txt" "$case_dir/adapter.txt" "$name"
    append_summary "$mode" "$repetition" "$case_dir/pipeline.txt"
}

if [[ ${1:-} == -h || ${1:-} == --help ]]; then
    usage
    exit 0
fi
if (( $# != 1 )); then
    usage >&2
    exit 2
fi
if (( EUID != 0 )); then
    echo "error: run as root so the harness can attach sched_ext" >&2
    exit 1
fi
bag_path=$1
if [[ $bag_path != /* || ! -e $bag_path ]]; then
    echo "error: bag path must be an existing absolute path" >&2
    exit 2
fi
if (( cpu == housekeeping_cpu )); then
    echo "error: CPU and HOUSEKEEPING_CPU must differ" >&2
    exit 2
fi
for value in "$cpu" "$housekeeping_cpu" "$duration" "$warmup" "$bag_offset" \
    "$repetitions" "$hog_threads" "$ext_policy"; do
    if [[ ! $value =~ ^[0-9]+$ ]]; then
        echo "error: numeric options must contain non-negative integers" >&2
        exit 2
    fi
done
if (( duration < 1 || warmup < 1 || repetitions < 1 )); then
    echo "error: DURATION, WARMUP, and REPETITIONS must be positive" >&2
    exit 2
fi
for topic in "$imu_topic" "$camera_topic"; do
    if [[ ! $topic =~ ^/[A-Za-z0-9_/]+$ ]]; then
        echo "error: topic names must be absolute and contain only letters, digits, _, and /" >&2
        exit 2
    fi
done
for file in "$loader_bin" "$pipeline_bin" "$adapter_bin"; do
    if [[ ! -x $file ]]; then
        echo "error: missing $file; run 'make && make ros2' as your normal user" >&2
        exit 1
    fi
done
for file in "$ros_setup" "$local_setup"; do
    if [[ ! -f $file ]]; then
        echo "error: missing ROS setup $file" >&2
        exit 1
    fi
done
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

mkdir -p "$output_dir" "$pin_dir"
trap cleanup EXIT INT TERM
write_qos_overrides
ros2 bag info "$bag_path" >"$output_dir/bag-info.txt"
if [[ -f $bag_path ]]; then
    bag_dir=$(dirname -- "$bag_path")
    bag_name=$(basename -- "$bag_path")
    (cd "$bag_dir" && sha256sum "$bag_name") >"$output_dir/bag-files.sha256"
else
    (
        cd "$bag_path"
        while IFS= read -r -d '' file; do
            sha256sum "$file"
        done < <(find . -type f -print0 | sort -z)
    ) >"$output_dir/bag-files.sha256"
fi
bag_sha256=$(sha256sum "$output_dir/bag-files.sha256" | awk '{print $1}')
worker_thread_siblings=$(< "/sys/devices/system/cpu/cpu${cpu}/topology/thread_siblings_list")
play_duration=$((warmup + duration + 2))
printf -v play_command \
    'ros2 bag play %q --rate 1.0 --start-offset %q --playback-duration %q --topics %q %q --qos-profile-overrides-path %q --delay 1 --disable-keyboard-controls --progress-bar-update-rate 0' \
    "$bag_path" "$bag_offset" "$play_duration" "$imu_topic" \
    "$camera_topic" "$output_dir/qos-overrides.yaml"

cat >"$output_dir/environment.txt" <<EOF
date=$(date --iso-8601=seconds)
kernel=$(uname -r)
kernel_cmdline=$(< /proc/cmdline)
ros_distro=${ROS_DISTRO:-unknown}
rmw_implementation=${RMW_IMPLEMENTATION:-default}
bag_path=$bag_path
bag_manifest_sha256=$bag_sha256
imu_topic=$imu_topic
camera_topic=$camera_topic
play_rate=1.0
play_clock=disabled
play_start_offset=$bag_offset
play_duration=$play_duration
play_delay=1
play_command=$play_command
qos=reliable,volatile,keep_last,depth_1000
cpu=$cpu
housekeeping_cpu=$housekeeping_cpu
worker_thread_siblings=$worker_thread_siblings
duration=$duration
warmup=$warmup
hog_threads=$hog_threads
ext_policy=$ext_policy
ops_flags=$ops_flags
repetitions=$repetitions
git_commit=$(git -c safe.directory="$repo_dir" -C "$repo_dir" rev-parse HEAD)
git_dirty=$(if [[ -n $(git -c safe.directory="$repo_dir" -C "$repo_dir" status --porcelain) ]]; then echo yes; else echo no; fi)
ros_pipeline_sha256=$(sha256sum "$(readlink -f "$pipeline_bin")" | awk '{print $1}')
ros_adapter_sha256=$(sha256sum "$(readlink -f "$adapter_bin")" | awk '{print $1}')
loader_sha256=$(sha256sum "$loader_bin" | awk '{print $1}')
bpf_object_sha256=$(sha256sum "$repo_dir/build/scx_slam_fresh.bpf.o" | awk '{print $1}')
EOF

printf "mode\trepetition\tstage\toffered\tcompleted\tlate\tstarted_stale\tunfinished\tcpu_us\tp99_start_age_us\tmax_start_age_us\tp99_age_us\tmax_age_us\tfirst_source_ts_ns\tlast_source_ts_ns\n" >"$output_dir/summary.tsv"

qos_preflight
for repetition in $(seq 1 "$repetitions"); do
    run_case cfs "$repetition"
done

taskset -c "$housekeeping_cpu" stdbuf -oL -eL "$loader_bin" --pin "$pin_dir" \
    >"$output_dir/loader.txt" 2>&1 &
loader_pid=$!
wait_for_scheduler
for repetition in $(seq 1 "$repetitions"); do
    run_case hinted "$repetition"
done

echo
echo "Bag-backed exploration complete; no comparative win is asserted."
echo "Results: $output_dir"
echo "Machine-readable metrics: $output_dir/summary.tsv"

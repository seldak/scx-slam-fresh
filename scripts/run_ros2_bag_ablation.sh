#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-2.0
# Three cells, identical binaries and work: control, downstream-BE, ordinary-FE IMU.
set -euo pipefail
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output_dir=${OUTPUT_DIR:-/tmp/scx-slam-bag-ablation-$(date +%Y%m%d-%H%M%S)-$$}
repetitions=${REPETITIONS:-3}
if (( $# != 1 || EUID != 0 )); then
    echo "usage: sudo scripts/run_ros2_bag_ablation.sh BAG" >&2
    exit 2
fi
if [[ ! $repetitions =~ ^[1-9][0-9]*$ || -e $output_dir ]]; then
    echo "error: require positive REPETITIONS and a new OUTPUT_DIR" >&2
    exit 2
fi
mkdir -p "$output_dir"
printf "cell\trepetition\tgate\n" >"$output_dir/gates.tsv"
gate_failed=0
baseline_dir=${BASELINE_DIR:-}
for variant in hinted imu-only fe-only; do
    for repetition in $(seq 1 "$repetitions"); do
        case_dir="$output_dir/$variant-$repetition"
        echo "=== Ablation $variant repetition $repetition ==="
        # Each child keeps set -e and the existing strict gate. The matrix may
        # retain a failed gate as an outcome; it never relabels it as a pass.
        if env HOG_THREADS=2 BE_SLICE_CAP_US=2000 HINTED_ONLY=1 \
            SCX_VARIANT="$variant" REPETITIONS=1 OUTPUT_DIR="$case_dir" \
            BASELINE_DIR="$baseline_dir" \
            "$repo_dir/scripts/run_ros2_bag_eval.sh" "$1" \
            >"$output_dir/$variant-$repetition.log" 2>&1; then
            gate=pass
        else
            gate=fail
            gate_failed=1
            if ! grep -q '^error: incomplete or inconsistent source-window accounting' \
                "$output_dir/$variant-$repetition.log"; then
                cat "$output_dir/$variant-$repetition.log"
                echo "error: infrastructure/input failure; stopping ablation" >&2
                exit 1
            fi
        fi
        cat "$output_dir/$variant-$repetition.log"
        printf "%s\t%s\t%s\n" "$variant" "$repetition" "$gate" >>"$output_dir/gates.tsv"
        for table in summary hog-summary; do
            if [[ ! -e $output_dir/$table.tsv ]]; then
                head -n 1 "$case_dir/$table.tsv" >"$output_dir/$table.tsv"
            fi
            awk -F '\t' -v OFS='\t' -v rep="$repetition" \
                'NR > 1 {$2 = rep; print}' "$case_dir/$table.tsv" >>"$output_dir/$table.tsv"
        done
        if [[ -z $baseline_dir ]]; then baseline_dir=$case_dir; fi
    done
done
# Check every cell, including rejected ones, against the first control window
# and require all runs to use the same binaries and pinned input/configuration.
python3 - "$output_dir" "$repetitions" <<'PY'
import csv
import pathlib
import sys
root = pathlib.Path(sys.argv[1])
repetitions = int(sys.argv[2])
windows = {}
cases = {}
with (root / "summary.tsv").open() as handle:
    for row in csv.DictReader(handle, delimiter="\t"):
        key = (row["mode"], int(row["repetition"]))
        stages = cases.setdefault(key, set())
        if row["stage"] in stages:
            raise SystemExit("error: duplicate stage in ablation cell")
        stages.add(row["stage"])
        sig = tuple(row[k] for k in ("offered", "first_source_ts_ns", "last_source_ts_ns",
                                    "first_job_id", "last_job_id"))
        if windows.setdefault(row["stage"], sig) != sig:
            raise SystemExit("error: unmatched ablation source windows")
        resolved = sum(int(row[k]) for k in
                       ("completed", "dropped_before_start", "dropped_upstream", "unfinished"))
        if resolved != int(row["offered"]):
            raise SystemExit("error: unaccounted ablation input")
expected_cases = {(mode, rep) for mode in ("hinted", "imu-only", "fe-only")
                  for rep in range(1, repetitions + 1)}
if set(cases) != expected_cases or any(stages != {"imu_prop", "vision_fe", "state_est", "mapping_be"}
                                     for stages in cases.values()):
    raise SystemExit("error: incomplete ablation matrix")
hog_cases = {}
with (root / "hog-summary.tsv").open() as handle:
    for row in csv.DictReader(handle, delimiter="\t"):
        key = (row["mode"], int(row["repetition"]))
        hogs = hog_cases.setdefault(key, {})
        hog = int(row["hog"])
        if hog in hogs or int(row["iteration_work_us"]) != 1000:
            raise SystemExit("error: invalid hog accounting")
        hogs[hog] = (int(row["start_ns"]), int(row["end_ns"]))
        if int(row["iterations"]) < 0:
            raise SystemExit("error: invalid iteration count")
if set(hog_cases) != expected_cases:
    raise SystemExit("error: missing hog accounting")
for hogs in hog_cases.values():
    if set(hogs) != {0, 1} or hogs[0] != hogs[1]:
        raise SystemExit("error: hog counters use different wall windows")
expected = None
keys = ("source_start_ns", "duration", "warmup", "hog_threads", "cpu", "housekeeping_cpu",
        "be_slice_cap_us", "deadline_grace_us", "ops_flags", "bag_manifest_sha256",
        "ros_pipeline_sha256", "ros_adapter_sha256", "loader_sha256", "bpf_object_sha256")
for path in sorted(root.glob("*/environment.txt")):
    env = dict(line.split("=", 1) for line in path.read_text().splitlines() if "=" in line)
    actual = tuple(env[k] for k in keys)
    if expected is None:
        expected = actual
    elif actual != expected:
        raise SystemExit("error: ablation binaries or configuration changed")
    # The driver fixes two hogs and a 2ms cap; all other policy settings match.
    if env["hog_threads"] != "2" or env["be_slice_cap_us"] != "2000":
        raise SystemExit("error: wrong ablation configuration")
    key = path.parent.name.rsplit("-", 1)
    window = hog_cases[(key[0], int(key[1]))][0]
    if window[1] - window[0] != int(env["duration"]) * 1_000_000_000:
        raise SystemExit("error: hog accounting duration differs from stage window")
print("Ablation input windows and binaries match across cells.")
PY
echo "Ablation results: $output_dir"
if (( gate_failed )); then
    echo "Ablation contains rejected strict gates; consult gates.tsv. No all-pass claim."
    exit 1
fi

#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fixed-window E4 sweep and isolated diagnostic A/Bs; no regime verdicts."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import random
import re
import signal
import shutil
import subprocess
import sys
import tempfile
import time


REPO = Path(__file__).resolve().parents[1]
STATE = Path("/sys/kernel/sched_ext")
STAGES = ("imu_prop", "vision_fe", "state_est", "lidar_pre", "lidar_reg", "mapping_be")
DEFAULT_COSTS = "150,500,1000,2000,3000,4000,4500,4750,5000,5500,6000"
DEFAULT_DEADLINE_GRACE_US = 1000
DEFAULT_LIDAR_PRE_BUDGET_US = 10000
DEFAULT_LIDAR_PRE_CLASS_ID = 1


def costs_arg(text):
    if not re.fullmatch(r"[0-9]+(?:,[0-9]+)*", text):
        raise argparse.ArgumentTypeError("costs must be comma-separated non-negative integers")
    costs = [int(value) for value in text.split(",")]
    if len(set(costs)) != len(costs) or 150 not in costs or max(costs) > 10000:
        raise argparse.ArgumentTypeError("use unique costs including 150, each at most 10000us")
    return costs


def case_plan(costs, repetitions, seed, probe=False, wakeup_only=False):
    rng = random.Random(seed)
    for repetition in range(1, repetitions + 1):
        middle = [cost for cost in costs if cost != 150]
        rng.shuffle(middle)
        for role, cost in [("control-start", 150), *[("sweep", c) for c in middle],
                           ("control-end", 150)]:
            modes = ["wakeup", "always"] if probe and not wakeup_only else ["wakeup"]
            rng.shuffle(modes)
            for mode in modes:
                yield {"name": f"r{repetition}-{mode}-{role}-{cost}", "repetition": repetition,
                       "role": role, "imu_work_us": cost, "imu_preempt": mode,
                       "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
                       "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
                       "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}


def grace_case_plan(repetitions, seed):
    rng = random.Random(seed)
    for repetition in range(1, repetitions + 1):
        variants = [DEFAULT_DEADLINE_GRACE_US, 0]
        rng.shuffle(variants)
        yield {"name": f"r{repetition}-wakeup-grace-{DEFAULT_DEADLINE_GRACE_US}-control-start-150",
               "repetition": repetition, "role": "control-start", "imu_work_us": 150,
               "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
               "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
               "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}
        for grace_us in variants:
            yield {"name": f"r{repetition}-wakeup-grace-{grace_us}-sweep-3500",
                   "repetition": repetition, "role": "sweep", "imu_work_us": 3500,
                   "imu_preempt": "wakeup", "deadline_grace_us": grace_us,
                   "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
                   "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}
        yield {"name": f"r{repetition}-wakeup-grace-{DEFAULT_DEADLINE_GRACE_US}-control-end-150",
               "repetition": repetition, "role": "control-end", "imu_work_us": 150,
               "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
               "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
               "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}


def lidar_pre_budget_case_plan(repetitions, seed):
    rng = random.Random(seed)
    for repetition in range(1, repetitions + 1):
        variants = [DEFAULT_LIDAR_PRE_BUDGET_US, 6000]
        rng.shuffle(variants)
        yield {"name": f"r{repetition}-wakeup-lpre-budget-{DEFAULT_LIDAR_PRE_BUDGET_US}-control-start-150",
               "repetition": repetition, "role": "control-start", "imu_work_us": 150,
               "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
               "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
               "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}
        for budget_us in variants:
            yield {"name": f"r{repetition}-wakeup-lpre-budget-{budget_us}-sweep-3500",
                   "repetition": repetition, "role": "sweep", "imu_work_us": 3500,
                   "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
                   "lidar_pre_budget_us": budget_us,
                   "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}
        yield {"name": f"r{repetition}-wakeup-lpre-budget-{DEFAULT_LIDAR_PRE_BUDGET_US}-control-end-150",
               "repetition": repetition, "role": "control-end", "imu_work_us": 150,
               "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
               "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
               "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}


def lidar_pre_class_case_plan(repetitions, seed):
    rng = random.Random(seed)
    for repetition in range(1, repetitions + 1):
        variants = [DEFAULT_LIDAR_PRE_CLASS_ID, 0]
        rng.shuffle(variants)
        yield {"name": f"r{repetition}-wakeup-lpre-class-fe-control-start-150",
               "repetition": repetition, "role": "control-start", "imu_work_us": 150,
               "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
               "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
               "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}
        for class_id in variants:
            label = "fe" if class_id == DEFAULT_LIDAR_PRE_CLASS_ID else "be"
            yield {"name": f"r{repetition}-wakeup-lpre-class-{label}-sweep-3500",
                   "repetition": repetition, "role": "sweep", "imu_work_us": 3500,
                   "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
                   "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
                   "lidar_pre_class_id": class_id}
        yield {"name": f"r{repetition}-wakeup-lpre-class-fe-control-end-150",
               "repetition": repetition, "role": "control-end", "imu_work_us": 150,
               "imu_preempt": "wakeup", "deadline_grace_us": DEFAULT_DEADLINE_GRACE_US,
               "lidar_pre_budget_us": DEFAULT_LIDAR_PRE_BUDGET_US,
               "lidar_pre_class_id": DEFAULT_LIDAR_PRE_CLASS_ID}


def command_for(case, args, pin_dir):
    build = args.binary_dir or REPO / "build"
    return ["taskset", "-c", str(args.cpu), str(build / "slam_pipeline_demo"),
            "--pin", str(pin_dir), "--ext-policy", "7", "--lidar", "heavy",
            "--hog", "0", "--drop-stale", "0", "--duration", str(args.duration),
            "--imu-work-us", str(case["imu_work_us"]), "--lidar-pre-budget-us",
            str(case.get("lidar_pre_budget_us", DEFAULT_LIDAR_PRE_BUDGET_US)),
            "--lidar-pre-class", "fe" if case.get("lidar_pre_class_id", DEFAULT_LIDAR_PRE_CLASS_ID) else "be",
            "--window-stats"]


def observer_prefix(cpu):
    others = sorted(os.sched_getaffinity(0) - {cpu})
    if not others:
        raise RuntimeError("diagnostic needs another allowed CPU for the observer")
    return ["taskset", "-c", ",".join(map(str, others))]


def perf_command(command, cpu, data_file):
    # All-CPU wake events are needed: wakeup can originate off the pinned CPU.
    # The workload resets its own affinity; only the perf reader stays off it.
    # Avoid `perf sched record`: its sched_stat_runtime stream is enormous for
    # these CPU-clock busy loops and perturbs the measurement it should observe.
    events = ("sched:sched_switch", "sched:sched_waking", "sched:sched_wakeup_new",
              "sched:sched_process_fork", "sched:sched_migrate_task")
    selectors = [value for event in events for value in ("-e", event)]
    return [*observer_prefix(cpu), "perf", "record", "-a", "-k", "mono", "-m", "1024",
            "-c", "1", *selectors, "-N", "-B", "--no-buildid-mmap",
            "-o", str(data_file), "--", *command]


def parse_metrics(text, cost, duration, expected_policy=7,
                  lidar_pre_budget_us=DEFAULT_LIDAR_PRE_BUDGET_US,
                  lidar_pre_class_id=DEFAULT_LIDAR_PRE_CLASS_ID):
    data = {}
    for line in text.splitlines():
        name, sep, rest = line.partition(":")
        if sep and name in (*STAGES, "generated", "configuration", "measurement", "imu_identity",
                            "focus_vision_fe", "focus_state_est",
                            *[f"window_{s}" for s in STAGES], *[f"drain_{s}" for s in STAGES]):
            if name in data:
                raise ValueError(f"duplicate metrics for {name}")
            data[name] = dict((k, int(v)) for k, v in
                              re.findall(r"\b([a-z_]+)=(\d+)(?=\s|$)", rest))
    expected_inputs = {"imu": duration * 200,
                       "camera": (duration * 1000000000 + 32999999) // 33000000,
                       "lidar": duration * 10}
    if data.get("generated") != expected_inputs:
        raise ValueError("offered sensor counts differ from the configured release schedule")
    config = data.get("configuration", {})
    for key, expected in {"imu_work_us": cost, "vision_work_us": 0,
                          "vision_budget_us": 12000, "vision_deadline_us": 33000,
                          "lidar_pre_budget_us": lidar_pre_budget_us,
                          "lidar_pre_class_id": lidar_pre_class_id}.items():
        if config.get(key) != expected:
            raise ValueError(f"unexpected configuration: {key}")
    for stage in STAGES:
        metrics = data.get(stage, {})
        required = {"processed", "late", "cpu_us"}
        if stage != "imu_prop":
            required |= {"dequeued", "pending", "stale_seen", "dropped_stale"}
        if not required <= metrics.keys():
            raise ValueError(f"missing metrics: {stage}")
        if metrics["late"] > metrics["processed"]:
            raise ValueError(f"late exceeds processed: {stage}")
        if stage != "imu_prop" and (metrics["dropped_stale"] != 0 or
                                     metrics["dequeued"] != metrics["processed"]):
            raise ValueError(f"unexpected dropping/dequeue accounting: {stage}")
    offered = {"imu_prop": expected_inputs["imu"], "vision_fe": expected_inputs["camera"],
               "state_est": data["vision_fe"]["processed"], "lidar_pre": expected_inputs["lidar"],
               "lidar_reg": data["lidar_pre"]["processed"],
               "mapping_be": data["state_est"]["processed"] + data["lidar_reg"]["processed"]}
    for stage in STAGES:
        m = data[stage]
        if m["processed"] + m.get("pending", 0) != offered[stage]:
            raise ValueError(f"offered/completed/pending do not reconcile: {stage}")
        m["offered"] = offered[stage]
    if data["imu_prop"]["cpu_us"] < expected_inputs["imu"] * cost:
        raise ValueError("IMU compute consumed less than the requested thread CPU time")
    timing = data.get("measurement", {})
    required = {"start_ns", "end_ns", "window_ns", "elapsed_ns", "drain_elapsed_ns"}
    if not required <= timing.keys():
        raise ValueError("missing fixed-window measurement; drain-inclusive logs are not accepted")
    if (timing["window_ns"] != duration * 1000000000 or
            timing["end_ns"] - timing["start_ns"] != timing["window_ns"] or
            timing["elapsed_ns"] != timing["window_ns"] + timing["drain_elapsed_ns"]):
        raise ValueError("inconsistent fixed-window epochs or drain elapsed time")
    identity = data.get("imu_identity", {})
    if identity.get("policy") != expected_policy or identity.get("stage_id") != 0 or not identity.get("pid_tgid"):
        raise ValueError("IMU identity/policy mismatch")
    for stage in STAGES:
        w, d, total = data.get(f"window_{stage}", {}), data.get(f"drain_{stage}", {}), data[stage]
        common = {"completed", "late", "cpu_us", "cpu_ns", "cpu_uncertainty_ns"}
        if not (common <= d.keys() and
                common | {"offered", "stale_seen", "dropped_stale", "pending", "in_flight"} <= w.keys()):
            raise ValueError(f"missing window/drain metrics: {stage}")
        if (w["offered"] != w["completed"] + w["pending"] + w["in_flight"] + w["dropped_stale"] or
                w["dropped_stale"] != 0 or w["in_flight"] > 1 or
                w["late"] > w["completed"] or d["late"] > d["completed"]):
            raise ValueError(f"inconsistent window queue/completion metrics: {stage}")
        if w["completed"] + d["completed"] != total["processed"] or w["late"] + d["late"] != total["late"]:
            raise ValueError(f"window/drain completions do not reconcile: {stage}")
        ns = w["cpu_ns"] + d["cpu_ns"]
        if (ns // 1000 != total["cpu_us"] or w["cpu_us"] != w["cpu_ns"] // 1000 or
                d["cpu_us"] != d["cpu_ns"] // 1000 or
                w["cpu_uncertainty_ns"] != d["cpu_uncertainty_ns"] or
                w["cpu_uncertainty_ns"] > d["cpu_ns"]):
            raise ValueError(f"window/drain CPU bounds do not reconcile: {stage}")
    if data["window_imu_prop"]["offered"] != expected_inputs["imu"]:
        raise ValueError("window IMU offered count does not match the epoch")
    sources = {"vision_fe": expected_inputs["camera"], "lidar_pre": expected_inputs["lidar"],
               "state_est": data["window_vision_fe"]["completed"],
               "lidar_reg": data["window_lidar_pre"]["completed"],
               "mapping_be": data["window_state_est"]["completed"] + data["window_lidar_reg"]["completed"]}
    for stage, source_count in sources.items():
        if data[f"window_{stage}"]["offered"] > source_count:
            raise ValueError(f"window deliveries exceed available source outputs: {stage}")
    if sum(data[f"window_{s}"]["cpu_ns"] for s in STAGES) > timing["window_ns"]:
        raise ValueError("per-stage window CPU exceeds one pinned core")
    for name in ("focus_vision_fe", "focus_state_est"):
        if name not in data:
            continue
        focus = data[name]
        required = {"job", "release_ns", "completion_ns", "age_ns", "late"}
        if (not required <= focus.keys() or focus["job"] != 4 or
                focus["completion_ns"] < focus["release_ns"] or
                focus["age_ns"] != focus["completion_ns"] - focus["release_ns"] or
                focus["late"] != int(focus["age_ns"] > 33_000_000)):
            raise ValueError(f"inconsistent focus job metrics: {name}")
    return data


def percentage(late, processed):
    return 100 * late / processed if processed else None


def ratio(completed, control):
    return completed / control if control else None


def summarize(cases):
    controls = {(c["repetition"], c["imu_preempt"]): c for c in cases if c["role"] == "control-start"}
    matrix, stages, drain = [], [], []
    for case in cases:
        data = case["metrics"]
        control_case = controls[(case["repetition"], case["imu_preempt"])]
        control = control_case["metrics"]
        row = {k: case[k] for k in ("name", "repetition", "role", "imu_work_us", "imu_preempt", "process_elapsed_s")}
        row["deadline_grace_us"] = case.get("deadline_grace_us", DEFAULT_DEADLINE_GRACE_US)
        row["lidar_pre_budget_us"] = case.get("lidar_pre_budget_us", DEFAULT_LIDAR_PRE_BUDGET_US)
        row["lidar_pre_class_id"] = case.get("lidar_pre_class_id", DEFAULT_LIDAR_PRE_CLASS_ID)
        timing = data["measurement"]
        window_s = timing["window_ns"] / 1e9
        row.update(control_case=control_case["name"], imu_nominal_pct=case["imu_work_us"] / 50,
                   observation="fixed-window", regime="unclassified-exploratory",
                   window_s=window_s, drain_elapsed_s=timing["drain_elapsed_ns"] / 1e9,
                   imu_window_cpu_pct=100 * data["window_imu_prop"]["cpu_ns"] / timing["window_ns"],
                   imu_total_cpu_pct=100 * data["imu_prop"]["cpu_us"] * 1000 / timing["elapsed_ns"],
                   camera_not_delivered_at_cutoff=data["generated"]["camera"] - data["window_vision_fe"]["offered"],
                   lidar_not_delivered_at_cutoff=data["generated"]["lidar"] - data["window_lidar_pre"]["offered"],
                   vision_to_estimator_not_delivered_at_cutoff=(
                       data["window_vision_fe"]["completed"] - data["window_state_est"]["offered"]))
        for stage in ("vision_fe", "state_est"):
            focus = data.get(f"focus_{stage}")
            row[f"{stage}_job4_completed"] = int(focus is not None)
            row[f"{stage}_job4_age_ns"] = focus["age_ns"] if focus else None
            row[f"{stage}_job4_late"] = focus["late"] if focus else None
        for stage in STAGES:
            m = data[f"window_{stage}"]
            unfinished = m["pending"] + m["in_flight"]
            miss_pct = percentage(m["late"], m["completed"])
            progress = ratio(m["completed"], control[f"window_{stage}"]["completed"])
            stages.append({"case": case["name"], "stage": stage, "offered": m["offered"],
                           "completed": m["completed"], "rate_hz": m["completed"] / window_s,
                           "late": m["late"], "miss_pct": miss_pct, "pending": m["pending"],
                           "in_flight": m["in_flight"], "unfinished": unfinished, "stale_seen": m["stale_seen"],
                           "dropped_stale": m["dropped_stale"], "cpu_us": m["cpu_us"],
                           "cpu_uncertainty_ns": m["cpu_uncertainty_ns"],
                           "rate_ratio_to_control": progress})
            drain.append({"case": case["name"], "stage": stage,
                          "drain_elapsed_s": row["drain_elapsed_s"], **data[f"drain_{stage}"]})
            for key in ("offered", "completed", "late", "pending", "in_flight", "stale_seen", "dropped_stale"):
                row[f"{stage}_{key}"] = m[key]
            row[f"{stage}_unfinished"] = unfinished
            row[f"{stage}_rate_hz"] = m["completed"] / window_s
            row[f"{stage}_miss_pct"] = miss_pct
            row[f"{stage}_cpu_us"] = m["cpu_us"]
            row[f"{stage}_cpu_uncertainty_ns"] = m["cpu_uncertainty_ns"]
            if stage in ("lidar_reg", "mapping_be"):
                row[f"{stage}_rate_ratio_to_control"] = progress
        matrix.append(row)
    return matrix, stages, drain


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def save_summaries(output, cases):
    matrix, stages, drain = summarize(cases)
    for name, rows in (("matrix.csv", matrix), ("stages.csv", stages), ("drain.csv", drain)):
        if rows:
            with (output / name).open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)
    return matrix


def format_case_summary(row):
    lines = [f"  WINDOW: {row['window_s']:g}s; deadline_grace_us={row['deadline_grace_us']}; "
             f"lidar_pre_budget_us={row['lidar_pre_budget_us']}; "
             f"lidar_pre_class={'fe' if row['lidar_pre_class_id'] else 'be'}; "
             f"control={row['control_case']}"]
    for stage in STAGES:
        line = (f"    {stage}: offered={row[f'{stage}_offered']} "
                f"completed={row[f'{stage}_completed']} late={row[f'{stage}_late']} "
                f"unfinished={row[f'{stage}_unfinished']} "
                f"(pending={row[f'{stage}_pending']} in_flight={row[f'{stage}_in_flight']}) "
                f"cpu_us={row[f'{stage}_cpu_us']}")
        if stage in ("lidar_reg", "mapping_be"):
            progress = row[f"{stage}_rate_ratio_to_control"]
            relative = f"{progress:.3f}x" if progress is not None else "n/a"
            line += f" rate/control={relative}"
        lines.append(line)
    lines.append(f"    vision->estimator: output_hz={row['vision_fe_rate_hz']:.3f} "
                 f"arrivals_hz={row['state_est_offered'] / row['window_s']:.3f} "
                 f"not_delivered_at_cutoff={row['vision_to_estimator_not_delivered_at_cutoff']}")
    lines.append("    focus job 4: " + " ".join(
        f"{stage}_age_us={row[f'{stage}_job4_age_ns'] / 1000:.3f} "
        f"{stage}_late={row[f'{stage}_job4_late']}"
        if row[f"{stage}_job4_completed"] else f"{stage}=not-completed"
        for stage in ("vision_fe", "state_est")))
    lines.append(f"  DRAIN: {row['drain_elapsed_s']:.3f}s (excluded above)")
    return "\n".join(lines)


def stop_process(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def read_state(name):
    return (STATE / name).read_text().strip()


def check_scheduler(loader, enable_seq):
    returncode = loader.poll()
    state = {name: read_state(name) for name in ("state", "enable_seq", "switch_all")}
    if (returncode is not None or state["state"] != "enabled" or
            state["enable_seq"] != enable_seq or state["switch_all"] != "0"):
        raise RuntimeError(f"scheduler check failed: loader_returncode={returncode}, "
                           f"state={state['state']}, enable_seq={state['enable_seq']} "
                           f"(expected {enable_seq}), switch_all={state['switch_all']} "
                           "(expected 0 for partial switch)")


def check_embedded_mode(loader_bin):
    text = subprocess.check_output([str(loader_bin), "--print-ops-flags"], text=True).strip()
    if not re.fullmatch(r"0x[0-9a-fA-F]+", text):
        raise RuntimeError(f"invalid embedded ops flags: {text!r}; rebuild the loader")
    flags = int(text, 16)
    # SCX_OPS_SWITCH_PARTIAL is bit 3 in the sched_ext ABI.
    if not flags & (1 << 3):
        raise RuntimeError(f"embedded ops_flags={text} lacks SCX_OPS_SWITCH_PARTIAL; "
                           "refusing to attach a full-switch scheduler; rebuild in partial mode")
    return text


def check_artifacts(hashes):
    for relative, expected in hashes.items():
        actual = hashlib.sha256((REPO / relative).read_bytes()).hexdigest()
        if actual != expected:
            raise RuntimeError(f"artifact changed during the run: {relative}; refusing a mixed-binary comparison")


def parse_trace(text, metrics, cpu, mode):
    events, summary = [], None
    timing, identity = metrics["measurement"], metrics["imu_identity"]
    for line in text.splitlines():
        if line.startswith("[imu_enqueue] ") or line.startswith("imu_trace_summary:"):
            fields = {k: int(v, 16 if v.startswith("0x") else 10) for k, v in
                      re.findall(r"\b([a-z_]+)=(0x[0-9a-fA-F]+|[0-9]+)(?=\s|$)", line)}
            if line.startswith("imu_trace_summary:"):
                if summary is not None:
                    raise ValueError("duplicate IMU trace summary")
                summary = fields
                continue
            required = {"ts_ns", "pid_tgid", "job", "stage", "release_ns", "deadline_ns",
                        "enq_flags", "dsq", "policy", "cpu", "wakeup", "late", "hint_present"}
            if not required <= fields.keys():
                raise ValueError("incomplete IMU enqueue sample")
            if (fields["wakeup"] != int(bool(fields["enq_flags"] & 1)) or
                    fields["late"] != int(bool(fields["deadline_ns"] and fields["ts_ns"] > fields["deadline_ns"]))):
                raise ValueError("inconsistent IMU wakeup/late indicators")
            if fields["pid_tgid"] != identity["pid_tgid"] or fields["cpu"] != cpu or fields["policy"] != 7:
                raise ValueError("IMU enqueue identity/CPU/policy mismatch")
            timestamp = fields["ts_ns"]
            fields["phase"] = ("startup" if timestamp < timing["start_ns"] else
                               "window" if timestamp < timing["end_ns"] else "drain")
            fields["relative_ns"] = timestamp - timing["start_ns"]
            # Decode the recorded destination, not an inferred dispatch lane.
            fields["route"] = {0x8000000000000002: "local-preempt", 0x1A01: "DSQ_IMU",
                               0xFE01: "DSQ_FE", 0xBE01: "DSQ_BE", 0x5A1E: "DSQ_STALE"}.get(fields["dsq"], "unknown")
            if fields["hint_present"] and fields["stage"] == 0:
                expected_route = "local-preempt" if mode == "always" or fields["wakeup"] else "DSQ_IMU"
                if fields["route"] != expected_route:
                    raise ValueError("IMU probe destination does not match the selected preemption mode")
            events.append(fields)
    required = {"enqueues", "wakeup", "nonwakeup", "late_wakeup", "late_nonwakeup", "local_preempt",
                "dsq_imu", "missing_hint", "wrong_stage", "wrong_policy", "emitted", "lost"}
    if summary is None or not required <= summary.keys():
        raise ValueError("missing IMU trace counters")
    if (summary["wakeup"] + summary["nonwakeup"] != summary["enqueues"] or
            summary["emitted"] != len(events) or not summary["enqueues"]):
        raise ValueError("IMU trace counters/events do not reconcile")
    return events, summary


def run_one_case(case, args, output):
    if read_state("state") != "disabled":
        raise RuntimeError("another scheduler became active; refusing to replace it")
    pin_dir = Path(tempfile.mkdtemp(prefix="scx_slam_fresh_e4_", dir="/sys/fs/bpf"))
    loader = demo = None
    try:
        build = args.binary_dir or REPO / "build"
        loader_command = ["stdbuf", "-oL", "-eL", str(build / "scx_slam_fresh_user"),
                          "--pin", str(pin_dir), "--imu-preempt", case["imu_preempt"],
                          "--deadline-grace-us", str(case.get("deadline_grace_us",
                                                               DEFAULT_DEADLINE_GRACE_US))]
        if args.preempt_probe:
            loader_command.append("--trace-imu")
        if args.perf_sched:
            loader_command = [*observer_prefix(args.cpu), *loader_command, "--trace-estimator"]
        if args.execution_probe:
            loader_command.extend(["--trace-execution-cpu", str(args.cpu)])
            # Keep the high-volume trace consumer off the measured CPU. This
            # is an observer affinity, not a workload/policy change.
            observer_cpus = sorted(os.sched_getaffinity(0) - {args.cpu})
            if not observer_cpus:
                raise RuntimeError("execution probe needs another allowed CPU for the trace consumer")
            loader_command = ["taskset", "-c", ",".join(map(str, observer_cpus)), *loader_command]
        command = command_for(case, args, pin_dir)
        if args.perf_sched:
            command = perf_command(command, args.cpu, output / f"{case['name']}.perf.data")
        write_json(output / f"{case['name']}.command.json", {"demo": command, "loader": loader_command})
        loader_file = output / f"{case['name']}.loader.txt"
        with loader_file.open("w") as loader_log:
            loader = subprocess.Popen(loader_command, stdout=loader_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            for _ in range(50):
                if loader.poll() is not None:
                    raise RuntimeError(f"loader exited before attach; inspect {loader_file.name}")
                if (pin_dir / "events").exists() and read_state("state") == "enabled":
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError("scheduler attach timed out")
            enable_seq = read_state("enable_seq")
            check_scheduler(loader, enable_seq)
            start = time.monotonic()
            timeout_s = max(30, args.duration * (1 + case["imu_work_us"] / 5000) * 3)
            with (output / f"{case['name']}.txt").open("w") as log, \
                    (output / f"{case['name']}.record.stderr.txt").open("w") as recorder_log:
                demo = subprocess.Popen(command, stdout=log,
                                        stderr=recorder_log if args.perf_sched else subprocess.STDOUT,
                                        start_new_session=True)
                while demo.poll() is None:
                    check_scheduler(loader, enable_seq)
                    if time.monotonic() - start > timeout_s:
                        raise RuntimeError(f"{case['name']} timed out; partial results retained")
                    time.sleep(0.1)
            elapsed = round(time.monotonic() - start, 3)
            check_scheduler(loader, enable_seq)
            if demo.returncode != 0:
                raise RuntimeError(f"{case['name']} exited with {demo.returncode}")
            text = (output / f"{case['name']}.txt").read_text()
            recorder_text = ((output / f"{case['name']}.record.stderr.txt").read_text()
                             if args.perf_sched else "")
            if "update failed:" in text + recorder_text:
                raise RuntimeError("hint publication failed; refusing mislabeled SCX results")
            metrics = parse_metrics(text, case["imu_work_us"], args.duration,
                                    lidar_pre_budget_us=case.get("lidar_pre_budget_us",
                                                                 DEFAULT_LIDAR_PRE_BUDGET_US),
                                    lidar_pre_class_id=case.get("lidar_pre_class_id",
                                                                DEFAULT_LIDAR_PRE_CLASS_ID))
            if ((args.grace_probe or args.lidar_pre_budget_probe or args.lidar_pre_class_probe) and
                    not {"focus_vision_fe", "focus_state_est"} <= metrics.keys()):
                raise RuntimeError("focused E4 probe requires completed vision and estimator job-4 metrics")
            case = dict(case, process_elapsed_s=elapsed, enable_seq=enable_seq, metrics=metrics)
            # A fresh attachment per case selects rodata without rebuilding,
            # resets unsampled counters, and prevents cross-case ring mixing.
            stop_process(loader)
            if loader.returncode != 0:
                raise RuntimeError(f"loader shutdown failed with {loader.returncode}")
        if args.preempt_probe:
            events, counters = parse_trace(loader_file.read_text(), case["metrics"], args.cpu, case["imu_preempt"])
            case["imu_trace_counters"] = counters
            case["imu_trace_complete"] = counters["lost"] == 0
            with (output / f"{case['name']}.imu-enqueues.csv").open("w", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(events[0]) if events else ["ts_ns"])
                writer.writeheader()
                writer.writerows(events)
            # These are sampled window events, NOT whole-window enqueue counts.
            window_events = [e for e in events if e["phase"] == "window"]
            case["imu_window_samples"] = {"samples": len(window_events),
                                          "late_wakeup": sum(e["late"] and e["wakeup"] for e in window_events),
                                          "late_nonwakeup": sum(e["late"] and not e["wakeup"] for e in window_events)}
        if args.execution_probe:
            from e4_execution import analyze_execution, parse_execution, write_csv
            events, counters = parse_execution(loader_file.read_text(), case["metrics"], args.cpu)
            write_csv(output / f"{case['name']}.execution.csv", events)
            summary, intervals, away, occupancy = analyze_execution(events, counters, case["metrics"])
            write_json(output / f"{case['name']}.execution-summary.json", summary)
            write_csv(output / f"{case['name']}.execution-intervals.csv", intervals)
            write_csv(output / f"{case['name']}.switch-away.csv", away)
            write_csv(output / f"{case['name']}.cpu-occupancy.csv", occupancy)
            if not summary["valid"]:
                raise RuntimeError(f"incomplete execution diagnostic: {summary['reason']}; raw capture retained")
            case["execution"] = summary
        return case
    finally:
        stop_process(demo)
        stop_process(loader)
        for name in ("task_hints", "events"):
            (pin_dir / name).unlink(missing_ok=True)
        pin_dir.rmdir()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cpu", type=int, default=0)
    parser.add_argument("--duration", type=int, default=15)
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--costs", type=costs_arg)
    parser.add_argument("--perf-sched", action="store_true",
                        help="standard perf sched plus estimator enqueue lanes; defaults to 3/65/70/3 percent")
    parser.add_argument("--grace-probe", action="store_true",
                        help="70%% default-grace versus zero-grace A/B with lean perf and estimator lanes")
    parser.add_argument("--lidar-pre-budget-probe", action="store_true",
                        help="70%% LiDAR-pre 10ms versus 6ms budget A/B with 1ms grace and lean perf")
    parser.add_argument("--lidar-pre-class-probe", action="store_true",
                        help="70%% LiDAR-pre FE versus BE class A/B with default budget/grace and lean perf")
    parser.add_argument("--preempt-probe", action="store_true",
                        help="pair wakeup/always preemption with IMU tracing (default costs: 150,2000,3000)")
    parser.add_argument("--execution-probe", action="store_true",
                        help="add switch/wakeup and IMU execution tracing; implies preempt probe, costs 150,3000")
    parser.add_argument("--wakeup-only", action="store_true",
                        help="restrict an opt-in diagnostic probe to the unchanged wakeup-only policy")
    parser.add_argument("--binary-dir", type=Path,
                        help="use archived prebuilt binaries; bypass make freshness check, retain hash/mode checks")
    parser.add_argument("--seed", type=int, default=4)
    parser.add_argument("--output", type=Path, help="new results directory (must not exist)")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    explicit_perf_sched = args.perf_sched
    focused_probes = (args.grace_probe, args.lidar_pre_budget_probe, args.lidar_pre_class_probe)
    if any(focused_probes):
        if args.costs or args.preempt_probe or args.execution_probe or args.wakeup_only or args.binary_dir:
            parser.error("focused E4 probes are fixed current-binary wakeup-policy A/Bs")
        if sum(focused_probes) != 1:
            parser.error("select only one focused E4 probe")
        args.perf_sched = True
    if args.execution_probe:
        args.preempt_probe = True
    if args.perf_sched and (args.preempt_probe or args.wakeup_only or args.binary_dir):
        parser.error("--perf-sched uses current binaries and wakeup policy; do not combine with other probes")
    if args.wakeup_only and not args.preempt_probe:
        parser.error("--wakeup-only requires --preempt-probe or --execution-probe; ordinary sweeps already use wakeup")
    if args.binary_dir:
        args.binary_dir = args.binary_dir.resolve()
    if args.duration < 1 or args.repetitions < 1:
        parser.error("duration and repetitions must be positive")
    if args.cpu not in os.sched_getaffinity(0):
        parser.error("CPU is not in the calling process's allowed affinity")
    costs = args.costs or costs_arg("150,3250,3500" if explicit_perf_sched else
                                   "150,3000" if args.execution_probe else
                                   "150,2000,3000" if args.preempt_probe else DEFAULT_COSTS)
    if args.grace_probe:
        plan = list(grace_case_plan(args.repetitions, args.seed))
    elif args.lidar_pre_budget_probe:
        plan = list(lidar_pre_budget_case_plan(args.repetitions, args.seed))
    elif args.lidar_pre_class_probe:
        plan = list(lidar_pre_class_case_plan(args.repetitions, args.seed))
    else:
        plan = list(case_plan(costs, args.repetitions, args.seed, args.preempt_probe, args.wakeup_only))
    if args.dry_run:
        for case in plan:
            command = command_for(case, args, "<unique-pin-dir>")
            if args.perf_sched:
                command = perf_command(command, args.cpu, f"<results>/{case['name']}.perf.data")
            print(case["name"], " ".join(command),
                  f"[loader: --imu-preempt {case['imu_preempt']}, trace_imu={int(args.preempt_probe)}, "
                  f"execution_cpu={args.cpu if args.execution_probe else -1}, "
                  f"deadline_grace_us={case['deadline_grace_us']}, "
                  f"lidar_pre_budget_us={case['lidar_pre_budget_us']}, "
                  f"lidar_pre_class={'fe' if case['lidar_pre_class_id'] else 'be'}]",
                  "[--trace-estimator; perf_sched=1]" if args.perf_sched else "")
        return 0
    if os.geteuid() != 0:
        parser.error("run with sudo to attach sched_ext; --dry-run needs no privileges")
    if read_state("state") != "disabled":
        parser.error("another sched_ext scheduler is active; refusing to replace it")
    if args.perf_sched:
        if not shutil.which("perf"):
            parser.error("perf is required for --perf-sched")
        observer_prefix(args.cpu)
    if args.binary_dir is None:
        subprocess.run(["make", "-q"], cwd=REPO, check=True)
    build = args.binary_dir or REPO / "build"
    for name in ("slam_pipeline_demo", "scx_slam_fresh_user", "scx_slam_fresh.bpf.o"):
        if not (build / name).is_file():
            parser.error(f"missing binary artifact: {build / name}")
    ops_flags = check_embedded_mode(build / "scx_slam_fresh_user")
    output = args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix="scx-e4-"))
    if args.output:
        output.mkdir(parents=True, exist_ok=False)
    else:
        output.chmod(0o755)  # Let the invoking non-root user inspect sudo results.
    print(f"Results: {output}\n{len(plan)} cases, {args.duration}s of releases each; drain can extend runs.",
          flush=True)
    artifacts = [str(build / name) for name in ("slam_pipeline_demo", "scx_slam_fresh_user", "scx_slam_fresh.bpf.o")]
    artifacts += ["scripts/run_e4_eval.py", "demo/window_metrics.h",
                 "scripts/e4_execution.py", "bpf/execution_trace.bpf.h", "include/scx_execution_trace.h"]
    if args.perf_sched:
        artifacts.append("scripts/e4_perf.py")
    environment = {"date": datetime.now(timezone.utc).isoformat(), "kernel": os.uname().release,
                   "cpu": args.cpu, "duration": args.duration, "seed": args.seed,
                   "plan": plan, "lidar": "heavy", "hogs": 0, "drop_stale": 0,
                   "ext_policy": 7, "ops_flags": ops_flags, "observation": "fixed-window",
                   "preempt_probe": args.preempt_probe, "execution_probe": args.execution_probe, "schema": 8,
                   "perf_sched": args.perf_sched,
                   "grace_probe": args.grace_probe,
                   "lidar_pre_budget_probe": args.lidar_pre_budget_probe,
                   "lidar_pre_class_probe": args.lidar_pre_class_probe,
                   "wakeup_only": args.wakeup_only, "binary_dir": str(build),
                   "source_tree_matches_binaries": args.binary_dir is None,
                   "binary_note": ("Archived binaries: current source.diff describes the runner's tree, "
                                   "not necessarily these binaries; inspect archived source snapshots if supplied."
                                   if args.binary_dir else "Current build passed make -q."),
                   "git_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO, text=True).strip(),
                   "git_status": subprocess.check_output(["git", "status", "--porcelain"], cwd=REPO, text=True),
                   "sha256": {p: hashlib.sha256((REPO / p).read_bytes()).hexdigest() for p in artifacts}}
    write_json(output / "environment.json", environment)
    if args.perf_sched:
        (output / "e4_perf.py").write_bytes((REPO / "scripts/e4_perf.py").read_bytes())
        (output / "perf-version.txt").write_bytes(subprocess.check_output(["perf", "version"]))
    (output / "runner.py").write_bytes(Path(__file__).read_bytes())
    (output / "window_metrics.h").write_bytes((REPO / "demo/window_metrics.h").read_bytes())
    for relative in ("scripts/e4_execution.py", "bpf/execution_trace.bpf.h", "include/scx_execution_trace.h"):
        (output / Path(relative).name).write_bytes((REPO / relative).read_bytes())
    (output / "source.diff").write_bytes(subprocess.check_output(["git", "diff", "HEAD"], cwd=REPO))
    (output / "cpu.txt").write_bytes(subprocess.check_output(["lscpu"]))
    if args.binary_dir:
        for name in ("scx_slam_fresh.bpf.c", "execution_trace.bpf.h"):
            source = build / name
            if source.is_file():
                (output / f"archived-{name}").write_bytes(source.read_bytes())
    cases = []
    try:
        for index, case in enumerate(plan, 1):
            check_artifacts(environment["sha256"])
            print(f"[{index}/{len(plan)}] {case['name']}", flush=True)
            case = run_one_case(case, args, output)
            check_artifacts(environment["sha256"])
            cases.append(case)
            write_json(output / "cases.json", cases)
            matrix = save_summaries(output, cases)
            print(format_case_summary(matrix[-1]), flush=True)
            if args.execution_probe:
                e = case["execution"]
                print("  IMU residency: " + ", ".join(
                    f"{label}={100 * e[key] / e['window_ns']:.2f}%" for label, key in
                    (("scheduled", "imu_switch_residency_ns"), ("blocked", "imu_blocked_ns"),
                     ("runnable-wait", "imu_runnable_wait_ns"), ("exit-path", "imu_exit_path_ns"))), flush=True)
        if args.perf_sched:
            from e4_perf import report_perf_cases
            print("Producing estimator lane and perf timehist reports...", flush=True)
            report_perf_cases(output, cases, args.cpu)
        write_json(output / "status.json", {"status": "complete-exploratory", "cases": len(cases)})
    except BaseException as error:
        write_json(output / "status.json", {"status": "incomplete", "error": str(error), "cases": len(cases)})
        raise
    print(f"Exploration captured: {output}. No regime thresholds or E4 pass claimed.", flush=True)
    return 0


def interrupted(signum, frame):
    raise KeyboardInterrupt(f"signal {signum}")


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupted)
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError, KeyboardInterrupt) as error:
        print(f"error: {error}", file=sys.stderr)
        sys.exit(1)

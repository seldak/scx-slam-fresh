#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Correlate estimator lane records with standard perf reports, not a switch tracer."""

import csv
import json
import re
import subprocess


LANES = {0xFE01: "DSQ_FE", 0xBE01: "DSQ_BE", 0x5A1E: "DSQ_STALE"}
HISTORICAL_MISS_OFFSET_NS = 135_052_000


def fields(line):
    return {key: int(value, 16 if value.startswith("0x") else 10)
            for key, value in re.findall(r"\b([a-z_]+)=(0x[0-9a-fA-F]+|[0-9]+)(?=\s|$)", line)}


def parse_estimator(text, metrics, cpu):
    enqueues, events, lidar_pre_events, counters = [], [], [], None
    start = metrics["measurement"]["start_ns"]
    end = metrics["measurement"]["end_ns"]
    for line in text.splitlines():
        if line.startswith("est_trace_summary:"):
            if counters is not None:
                raise ValueError("duplicate estimator trace summary")
            counters = fields(line)
        elif line.startswith("[est_enqueue] "):
            e = fields(line)
            required = {"ts_ns", "pid_tgid", "job", "stage", "release_ns", "deadline_ns", "enq_flags",
                        "dsq", "slice_ns", "vruntime", "exec_ns", "policy", "cpu", "overrun", "state_present"}
            if not required <= e.keys():
                raise ValueError("incomplete estimator enqueue record")
            if e["stage"] != 2 or e["policy"] != 7 or e["cpu"] != cpu or e["dsq"] not in LANES:
                raise ValueError("estimator enqueue identity/policy/CPU/lane mismatch")
            e.update(lane=LANES[e["dsq"]], relative_ns=e["ts_ns"] - start,
                     phase="startup" if e["ts_ns"] < start else "window" if e["ts_ns"] < end else "drain")
            enqueues.append(e)
        elif line.startswith("[evt] "):
            e = fields(line)
            e["kind"] = line.split()[1]
            if e.get("stage") == 5:
                if not {"ts_ns", "release_ns", "deadline_ns", "pid_tgid", "job"} <= e.keys():
                    raise ValueError("LiDAR-pre events need exact timestamps; rebuild the loader")
                e.update(relative_ns=e["ts_ns"] - start,
                         deadline_delta_ns=(e["ts_ns"] - e["deadline_ns"]
                                            if e["deadline_ns"] else None),
                         phase="startup" if e["ts_ns"] < start else
                         "window" if e["ts_ns"] < end else "drain")
                lidar_pre_events.append(e)
                continue
            if e.get("stage") != 2:
                continue
            if not {"ts_ns", "release_ns", "deadline_ns", "pid_tgid", "job"} <= e.keys():
                raise ValueError("estimator events need exact timestamps; rebuild the loader")
            e.update(relative_ns=e["ts_ns"] - start,
                     phase="startup" if e["ts_ns"] < start else "window" if e["ts_ns"] < end else "drain")
            events.append(e)
    if counters is None or not {"enqueues", "emitted", "lost"} <= counters.keys():
        raise ValueError("missing estimator trace counters")
    if (not enqueues or counters["emitted"] != len(enqueues) or counters["lost"] != 0 or
            counters["enqueues"] != counters["emitted"] + counters["lost"]):
        raise ValueError("incomplete estimator lane stream; raw capture retained")
    keys = {e["pid_tgid"] for e in enqueues + events}
    if len(keys) != 1 or not next(iter(keys)):
        raise ValueError("estimator identity changed")
    if any(b["ts_ns"] < a["ts_ns"] for a, b in zip(enqueues, enqueues[1:])):
        raise ValueError("estimator enqueue timestamps are not ordered")
    key = next(iter(keys))
    if key >> 32 != metrics["imu_identity"]["pid_tgid"] >> 32:
        raise ValueError("estimator and IMU belong to different processes")
    if not any(e["phase"] == "window" for e in enqueues):
        raise ValueError("no estimator enqueues in the observation window")
    first = {}
    for kind in ("DEADLINE_MISS", "STALE_DEMOTION"):
        matches = [e for e in events if e["kind"] == kind and e["phase"] == "window"]
        first[kind] = min(matches, key=lambda e: e["ts_ns"]) if matches else None
    return {"tid": key & 0xffffffff, "pid_tgid": key, "counters": counters,
            "first_events": first, "enqueues": enqueues, "events": events,
            "lidar_pre_events": lidar_pre_events}


def report_bounds(metrics, anchor_offset_ns):
    timing = metrics["measurement"]
    return (timing["start_ns"] + max(0, anchor_offset_ns - 100_000_000),
            min(timing["end_ns"], timing["start_ns"] + anchor_offset_ns + 2_000_000_000))


def seconds(ns):
    return f"{ns // 1_000_000_000}.{ns % 1_000_000_000:09d}"


def timehist_command(data_file, begin, end, *, tid=None, cpu=None):
    command = ["perf", "sched", "timehist", "-i", str(data_file), "--state", "-w", "-n",
               "--time", f"{seconds(begin)},{seconds(end)}"]
    if tid is not None:
        command += ["--tid", str(tid)]
    if cpu is not None:
        command += ["--cpu", str(cpu)]
    return command


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        keys = list(dict.fromkeys(key for row in rows for key in row))
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def run_report(command, path):
    with path.open("w") as out, path.with_suffix(path.suffix + ".stderr").open("w") as err:
        subprocess.run(command, stdout=out, stderr=err, check=True, timeout=60)


def perf_loss_reported(text):
    return bool(re.search(r"PERF_RECORD_LOST|LOST_SAMPLES|\blost\s+[1-9][0-9]*\b|\b[1-9][0-9]*\s+lost\b",
                          text, re.I))


def report_perf_cases(output, cases, cpu):
    parsed = {c["name"]: parse_estimator((output / f"{c['name']}.loader.txt").read_text(), c["metrics"], cpu)
              for c in cases}
    anchors = {}
    for c in cases:
        miss = parsed[c["name"]]["first_events"]["DEADLINE_MISS"]
        if (c["imu_work_us"] == 3500 and
                c.get("deadline_grace_us", 1000) == 1000 and
                c.get("lidar_pre_budget_us", 10000) == 10000 and
                c.get("lidar_pre_class_id", 1) == 1 and miss):
            anchors[c["repetition"]] = (miss["relative_ns"], c["name"])
    for c in cases:
        name = c["name"]
        data = parsed[name]
        offset, anchor_case = anchors.get(c["repetition"],
                                          (HISTORICAL_MISS_OFFSET_NS, "historical-offset-no-observed-70pct-miss"))
        begin, end = report_bounds(c["metrics"], offset)
        enqueues = data.pop("enqueues")
        events = data.pop("events")
        lidar_pre_events = data.pop("lidar_pre_events")
        miss = data["first_events"]["DEADLINE_MISS"]
        before_miss = max((e for e in enqueues if miss and e["ts_ns"] <= miss["ts_ns"]),
                          key=lambda e: e["ts_ns"], default=None)
        after_miss = min((e for e in enqueues if miss and e["ts_ns"] >= miss["ts_ns"]),
                         key=lambda e: e["ts_ns"], default=None)
        focus_enqueues = [e for e in enqueues if begin <= e["ts_ns"] < end]
        job4_enqueues = [e for e in enqueues if e["phase"] == "window" and e["job"] == 4]
        post_job4 = [e for e in focus_enqueues if e["job"] >= 5]
        first_lane_by_job = {}
        lanes_by_job = {}
        for enqueue in post_job4:
            first_lane_by_job.setdefault(str(enqueue["job"]), enqueue["lane"])
            lanes_by_job.setdefault(str(enqueue["job"]), set()).add(enqueue["lane"])
        non_fe_jobs = sorted(int(job) for job, lanes in lanes_by_job.items()
                             if lanes != {"DSQ_FE"})
        first_budget_overrun = min((e for e in lidar_pre_events
                                    if e["kind"] == "BUDGET_OVERRUN" and e["phase"] == "window"),
                                   key=lambda e: e["ts_ns"], default=None)
        first_budget_demotion = min((e for e in lidar_pre_events
                                     if e["kind"] == "BUDGET_DEMOTION" and e["phase"] == "window"),
                                    key=lambda e: e["ts_ns"], default=None)
        write_csv(output / f"{name}.estimator-enqueues.csv", enqueues)
        write_csv(output / f"{name}.estimator-events.csv", events)
        write_csv(output / f"{name}.lidar-pre-events.csv", lidar_pre_events)
        write_csv(output / f"{name}.estimator-focus.csv", focus_enqueues)
        data_file = output / f"{name}.perf.data"
        script_file = output / f"{name}.perf-focus.txt"
        commands = [
            (["perf", "script", "-i", str(data_file), "--ns", "--show-lost-events",
              "--time", f"{seconds(begin)},{seconds(end)}",
              "-F", "comm,pid,tid,cpu,time,event,trace"], script_file),
            (timehist_command(data_file, begin, end, tid=data["tid"]), output / f"{name}.estimator-timehist.txt"),
            (timehist_command(data_file, begin, end, cpu=cpu), output / f"{name}.cpu-timehist.txt"),
        ]
        metadata = dict(data, anchor_case=anchor_case, anchor_offset_ns=offset,
                        report_start_ns=begin, report_end_ns=end,
                        commands=[cmd for cmd, _ in commands],
                        attribution="enqueue destination, not dequeue provenance; perf supplies execution/wait",
                        ordinary_event_loss="not counted; first_events means first observed, not proven earliest")
        metadata["enqueue_before_first_miss"] = before_miss
        metadata["enqueue_after_first_miss"] = after_miss
        metadata["focus_lane_counts"] = {lane: sum(e["lane"] == lane for e in focus_enqueues)
                                          for lane in LANES.values()}
        metadata["focus_jobs"] = {
            stage: c["metrics"].get(f"focus_{stage}") for stage in ("vision_fe", "state_est")
        }
        metadata["estimator_job4"] = {
            "enqueue_lane_counts": {lane: sum(e["lane"] == lane for e in job4_enqueues)
                                    for lane in LANES.values()},
            "deadline_miss_observed": any(e["kind"] == "DEADLINE_MISS" and e["job"] == 4
                                          for e in events),
            "stale_demotion_observed": any(e["kind"] == "STALE_DEMOTION" and e["job"] == 4
                                           for e in events),
        }
        metadata["estimator_jobs_5_plus"] = {
            "observed_jobs": len(lanes_by_job),
            "first_lane_by_job": first_lane_by_job,
            "enqueue_lane_counts": {lane: sum(e["lane"] == lane for e in post_job4)
                                    for lane in LANES.values()},
            "jobs_with_non_fe_lane": non_fe_jobs,
            "all_observed_enqueues_fe": (not non_fe_jobs) if lanes_by_job else None,
        }
        metadata["lidar_pre_budget_path"] = {
            "first_budget_overrun": first_budget_overrun,
            "first_budget_demotion": first_budget_demotion,
            "demotion_destination": "DSQ_BE" if first_budget_demotion else None,
            "destination_basis": ("BUDGET_DEMOTION is emitted immediately before the policy's DSQ_BE insertion"
                                  if first_budget_demotion else None),
        }
        lidar_pre_class_id = c.get("lidar_pre_class_id", 1)
        metadata["lidar_pre_initial_class"] = {
            "class_id": lidar_pre_class_id,
            "class": "FE" if lidar_pre_class_id == 1 else "BE",
            "destination": "DSQ_FE" if lidar_pre_class_id == 1 else "DSQ_BE",
            "destination_basis": "configured hint class and scheduler class-routing branch",
        }
        for command, path in commands:
            run_report(command, path)
        loss = False
        saw_switch = False
        for path in [script_file, *(path.with_suffix(path.suffix + ".stderr") for _, path in commands),
                     output / f"{name}.record.stderr.txt"]:
            with path.open() as handle:
                for line in handle:
                    loss |= perf_loss_reported(line)
                    if path == script_file:
                        saw_switch |= "sched:sched_switch:" in line
        metadata["perf_loss_reported"] = loss
        metadata["perf_switch_events_present"] = saw_switch
        (output / f"{name}.estimator-focus.json").write_text(json.dumps(metadata, indent=2) + "\n")
        if loss or not saw_switch:
            raise ValueError(f"{name}: perf loss or missing switch data; do not attribute gaps")
        vision_focus = c["metrics"].get("focus_vision_fe")
        estimator_focus = c["metrics"].get("focus_state_est")
        print(f"  {name}: estimator tid={data['tid']}; "
              f"vision job4 age_ns={vision_focus['age_ns'] if vision_focus else 'not-completed'}; "
              f"estimator job4 age_ns={estimator_focus['age_ns'] if estimator_focus else 'not-completed'} "
              f"late={estimator_focus['late'] if estimator_focus else 'n/a'}; "
              f"first observed miss job/offset_ns="
              f"{str(miss['job']) + '/' + str(miss['relative_ns']) if miss else 'none'}; "
              f"lane before/after={before_miss['lane'] if before_miss else 'n/a'}/"
              f"{after_miss['lane'] if after_miss else 'n/a'}; "
              f"jobs5+ non-FE={len(non_fe_jobs)}/{len(lanes_by_job)}; "
              f"lpre initial class/destination="
              f"{metadata['lidar_pre_initial_class']['class']}/"
              f"{metadata['lidar_pre_initial_class']['destination']}; "
              f"lpre first budget demotion job/offset_ns="
              f"{str(first_budget_demotion['job']) + '/' + str(first_budget_demotion['relative_ns']) if first_budget_demotion else 'none'}; "
              f"lpre demotion deadline_delta_ns="
              f"{first_budget_demotion['deadline_delta_ns'] if first_budget_demotion else 'n/a'}; "
              f"focus={seconds(begin)}..{seconds(end)}", flush=True)

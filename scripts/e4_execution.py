# SPDX-License-Identifier: MIT
"""Loss-checked execution timelines. Synthetic tests are not E4 evidence."""

from collections import defaultdict
import csv
import re


KINDS = {1: "switch", 2: "wakeup", 3: "running", 4: "stopping"}
TASK_FIELDS = ("key", "policy", "stage", "slice_ns", "last_dsq", "insert_ns", "requested_slice_ns")
EVENT_FIELDS = ("kind", "ts_ns", "urgent_key", "cpu", "runnable", "preempt", "prev_state",
                "callback_delta_ns", "syscall_id")


def phase(timestamp, timing):
    return ("startup" if timestamp < timing["start_ns"] else
            "window" if timestamp < timing["end_ns"] else "drain")


def parse_execution(text, metrics, cpu):
    events, counters = [], None
    timing = metrics["measurement"]
    imu = metrics["imu_identity"]["pid_tgid"]
    for line in text.splitlines():
        if line.startswith("execution_summary:"):
            if counters is not None:
                raise ValueError("duplicate execution counters")
            counters = {k: int(v) for k, v in re.findall(r"([a-z_]+)=(\d+)(?=\s|$)", line)}
        if not line.startswith("[execution] "):
            continue
        fields = dict(part.split("=", 1) for part in line.split()[1:])
        if "imu_key" in fields and "urgent_key" not in fields:
            fields["urgent_key"] = fields.pop("imu_key")
        required = {*EVENT_FIELDS, "syscall",
                    *(f"{prefix}_{k}" for prefix in ("task", "next") for k in (*TASK_FIELDS, "comm_hex"))}
        if set(fields) != required:
            raise ValueError("incomplete or unknown execution record fields")
        event = {k: int(fields[k]) for k in EVENT_FIELDS}
        for prefix in ("task", "next"):
            event.update({f"{prefix}_{k}": int(fields[f"{prefix}_{k}"]) for k in TASK_FIELDS})
            encoded = fields[f"{prefix}_comm_hex"]
            if not re.fullmatch(r"[0-9a-f]{32}", encoded):
                raise ValueError("invalid execution comm encoding")
            event[f"{prefix}_comm_hex"] = encoded
            event[f"{prefix}_comm"] = bytes.fromhex(encoded).split(b"\0", 1)[0].decode("utf-8", errors="backslashreplace")
        if min(v for v in event.values() if isinstance(v, int)) < 0:
            raise ValueError("negative execution field")
        if event["kind"] not in KINDS or event["runnable"] not in (0, 1) or event["preempt"] not in (0, 1):
            raise ValueError("invalid execution kind/state")
        event["event"] = KINDS[event["kind"]]
        event["syscall"] = fields["syscall"]
        if event["syscall"] not in ("none", "clock_nanosleep", "nanosleep", "futex", "exit", "other"):
            raise ValueError("invalid syscall label")
        event["phase"] = phase(event["ts_ns"], timing)
        event["relative_ns"] = event["ts_ns"] - timing["start_ns"]
        if event["urgent_key"] not in (0, imu) or (event["phase"] == "window" and event["urgent_key"] != imu):
            raise ValueError("execution IMU identity mismatch")
        if event["kind"] == 1:
            if event["cpu"] != cpu:
                raise ValueError("execution switch on the wrong CPU")
            if event["runnable"] != int(bool(event["preempt"] or not event["prev_state"])):
                raise ValueError("execution switch runnable flag contradicts raw state/preempt")
        elif event["task_key"] != imu:
            raise ValueError("execution wakeup/callback belongs to another worker")
        # Wakeups may originate remotely. Running/stopping may not migrate.
        if event["kind"] in (3, 4) and event["cpu"] != cpu:
            raise ValueError("IMU execution migrated off the pinned CPU")
        for prefix in ("task", "next"):
            if event[f"{prefix}_key"] == imu and event["phase"] == "window":
                if event[f"{prefix}_policy"] != 7 or event[f"{prefix}_stage"] != 0:
                    raise ValueError("execution IMU policy/stage mismatch")
        events.append(event)
    if counters is None or set(counters) != {"emitted", "lost", "identity_conflicts"}:
        raise ValueError("missing execution counters")
    if counters["emitted"] != len(events) or not events:
        raise ValueError("execution counters/events do not reconcile")
    # Wakeups can be emitted on a different CPU; merge by monotonic timestamp.
    events.sort(key=lambda e: e["ts_ns"])
    return events, counters


def clipped(a, b, timing):
    return max(0, min(b, timing["end_ns"]) - max(a, timing["start_ns"]))


def analyze_execution(events, counters, metrics):
    timing = metrics["measurement"]
    start, end = timing["start_ns"], timing["end_ns"]
    imu = metrics["imu_identity"]["pid_tgid"]
    invalid = {"valid": False, "reason": "", "counters": counters}
    if counters["lost"] or counters["identity_conflicts"]:
        invalid["reason"] = "ring loss or ambiguous IMU identity; no gap attribution"
        return invalid, [], [], []
    switches = [e for e in events if e["kind"] == 1]
    if not switches or switches[0]["ts_ns"] > start or switches[-1]["ts_ns"] < end:
        invalid["reason"] = "switch trace does not bracket the fixed window"
        return invalid, [], [], []
    if any(a["next_key"] != b["task_key"] for a, b in zip(switches, switches[1:])):
        invalid["reason"] = "CPU switch chain is discontinuous"
        return invalid, [], [], []

    # A blocked switch-out ends at wakeup, NOT at the next running callback.
    # The remaining wakeup-to-switch-in interval is runnable waiting.
    states = []
    state, reason, previous = "unknown", "unknown", events[0]["ts_ns"]
    wake_while_running = False
    for e in events:
        timestamp = e["ts_ns"]
        if e["kind"] == 4:
            # A prior wake without an actual switch can be a completed past-
            # deadline sleep. Only a wake racing this stop/switch boundary is
            # ambiguous; do not carry that old wake into a later sleep.
            wake_while_running = False
        if e["kind"] not in (1, 2):
            continue
        if timestamp > previous:
            states.append((previous, timestamp, state, reason))
        previous = timestamp
        if e["kind"] == 2:
            if state == "on_cpu":
                wake_while_running = True
            else:
                state, reason = "runnable_wait", "none"
        elif e["next_key"] == imu:
            state, reason = "on_cpu", "none"
            wake_while_running = False
        elif e["task_key"] == imu:
            if e["runnable"]:
                state, reason = "runnable_wait", "none"
            elif wake_while_running:
                # A cross-CPU wake raced switch-out; do not invent its ordering.
                state, reason = "unknown", "wake_switch_race"
            elif e["syscall"] == "exit":
                # Underloaded IMU can finish its final tick just before T.
                # Do not label its off-CPU termination path as sleep starvation.
                state, reason = "exit_path", "exit"
            else:
                state, reason = "blocked", e["syscall"]
            wake_while_running = False
    states.append((previous, max(end, previous), state, reason))

    # Intersect the IMU state timeline with ALL CPU switches, not only the
    # immediate successor. Several tasks may run during one off-CPU gap.
    intervals, occupancy = [], defaultdict(int)
    pos = 0
    for a, b in zip(switches, switches[1:]):
        left, right = max(start, a["ts_ns"]), min(end, b["ts_ns"])
        if left >= right:
            continue
        while pos < len(states) and states[pos][1] <= left:
            pos += 1
        cursor = pos
        while cursor < len(states) and states[cursor][0] < right:
            sa, sb, status, why = states[cursor]
            lo, hi = max(left, sa), min(right, sb)
            if lo < hi:
                row = {"start_ns": lo, "end_ns": hi, "duration_ns": hi - lo,
                       "imu_state": status, "blocked_syscall": why,
                       **{f"cpu_{k}": a[f"next_{k}"] for k in (*TASK_FIELDS, "comm")}}
                intervals.append(row)
                occupancy[(status, why, a["next_key"], a["next_comm"], a["next_policy"],
                           a["next_stage"], a["next_last_dsq"])] += hi - lo
            cursor += 1
    totals = {s: sum(r["duration_ns"] for r in intervals if r["imu_state"] == s)
              for s in ("on_cpu", "blocked", "runnable_wait", "exit_path", "unknown")}
    if sum(totals.values()) != end - start or totals["unknown"]:
        invalid["reason"] = "window contains uncovered or ambiguous IMU state"
        return invalid, intervals, [], []
    if any((r["cpu_key"] == imu) != (r["imu_state"] == "on_cpu") for r in intervals):
        invalid["reason"] = "IMU state contradicts observed CPU occupant"
        return invalid, intervals, [], []

    # Scheduler callbacks can end/restart without a real context switch. Keep
    # this independent from the sched_switch residency measurement.
    callback_intervals, running = [], None
    for e in events:
        if e["kind"] == 3:
            if running is not None:
                invalid["reason"] = "unpaired IMU running callbacks"
                return invalid, intervals, [], []
            running = e
        elif e["kind"] == 4:
            if running is None:
                if e["ts_ns"] >= start:
                    invalid["reason"] = "unpaired IMU stopping callback"
                    return invalid, intervals, [], []
            else:
                callback_intervals.append((running["ts_ns"], e["ts_ns"]))
            running = None
    if running is not None and running["ts_ns"] < end:
        invalid["reason"] = "IMU callback coverage ends before cutoff"
        return invalid, intervals, [], []
    callback_ns = sum(clipped(a, b, timing) for a, b in callback_intervals)
    if not callback_ns:
        invalid["reason"] = "no IMU running/stopping coverage in window"
        return invalid, intervals, [], []

    away = []
    for i, e in enumerate(switches[:-1]):
        if e["task_key"] != imu:
            continue
        prior = switches[i - 1] if i else None
        after = switches[i + 1]
        away.append({"ts_ns": e["ts_ns"], "phase": e["phase"], "runnable": e["runnable"],
                     "prev_state": e["prev_state"], "preempt": e["preempt"], "syscall": e["syscall"],
                     "imu_residency_ns": e["ts_ns"] - prior["ts_ns"] if prior else None,
                     "imu_remaining_slice_ns": e["task_slice_ns"],
                     "imu_start_slice_ns": prior["next_slice_ns"] if prior else None,
                     "next_residency_ns": after["ts_ns"] - e["ts_ns"],
                     "next_window_residency_ns": clipped(e["ts_ns"], after["ts_ns"], timing),
                     **{f"next_{k}": e[f"next_{k}"] for k in (*TASK_FIELDS, "comm")}})
    rows = [{"imu_state": k[0], "blocked_syscall": k[1], "pid_tgid": k[2], "comm": k[3],
             "policy": k[4], "stage": k[5], "last_insert_dsq": k[6], "window_ns": ns,
             "window_pct": ns * 100 / (end - start)} for k, ns in occupancy.items()]
    rows.sort(key=lambda r: -r["window_ns"])
    summary = {"valid": True, "counters": counters, "window_ns": end - start,
               "imu_switch_residency_ns": totals["on_cpu"], "imu_blocked_ns": totals["blocked"],
               "imu_runnable_wait_ns": totals["runnable_wait"], "imu_unknown_ns": totals["unknown"],
               "imu_exit_path_ns": totals["exit_path"],
               "imu_callback_residency_ns": callback_ns,
               "imu_compute_cpu_ns": metrics["window_imu_prop"]["cpu_ns"],
               "interpretation": "Residency is elapsed scheduled time, not IRQ-subtracted thread CPU; "
               "last_insert_dsq is routing provenance, not a dequeue trace. No cause/regime verdict."}
    return summary, intervals, away, rows


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["no_records"])
        writer.writeheader()
        writer.writerows(rows)

#!/usr/bin/env python3
"""Read-only terminal watcher for a running SIM control autotune process.

Only files under the selected results directory and Linux ``/proc`` are read.
This program does not import ROS, create publishers, or signal/control the
autotune process.
"""

from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple


DEFAULT_RESULTS_DIR = Path("/tmp/amet_autotune")
DEFAULT_INTERVAL_S = 20.0
CONFIG_FIELDS = (
    "min_lookahead",
    "max_lookahead",
    "curvature_reference",
    "preview_distance",
    "max_lateral_acceleration",
    "pid_enabled",
    "kp",
    "ki",
    "kd",
)
PARAM_LABELS = {
    "min_lookahead": "minLA",
    "max_lookahead": "maxLA",
    "curvature_reference": "curvRef",
    "preview_distance": "preview",
    "max_lateral_acceleration": "maxLatAcc",
    "pid_enabled": "PID",
    "kp": "kp",
    "ki": "ki",
    "kd": "kd",
}
RUN_START_PATTERN = re.compile(r"RUN\s+(\d+)\s+START(?:\s+PID STEP)?\s*(\S+)?")
RUN_END_PATTERN = re.compile(r"RUN\s+(\d+)\s+END")
PHASE_PATTERN = re.compile(r"PHASE\s+(\d+)\s+(.+)")


def read_jsonl(path: Path) -> List[Dict[str, Any]]:
    records: List[Dict[str, Any]] = []
    try:
        with path.open("r", encoding="utf-8") as source:
            for line in source:
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    # The writer may be between write and fsync; ignore only the
                    # currently incomplete line and retry on the next cycle.
                    continue
                if isinstance(value, dict):
                    records.append(value)
    except FileNotFoundError:
        pass
    return records


def read_json(path: Path) -> Optional[Dict[str, Any]]:
    try:
        with path.open("r", encoding="utf-8") as source:
            value = json.load(source)
        return value if isinstance(value, dict) else None
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


def tail_lines(path: Path, count: int) -> List[str]:
    try:
        with path.open("r", encoding="utf-8", errors="replace") as source:
            return [line.rstrip() for line in deque(source, maxlen=count)]
    except FileNotFoundError:
        return []


def process_cmdline(pid: int) -> List[str]:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return []
    return [part.decode("utf-8", "replace") for part in raw.split(b"\0") if part]


def is_autotune_alive(pid: int) -> bool:
    return any(
        argument.endswith("sim_control_autotune.py")
        for argument in process_cmdline(pid)
    )


def find_autotune_pid() -> Optional[int]:
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid != os.getpid() and is_autotune_alive(pid):
            return pid
    return None


def process_elapsed_s(pid: int) -> Optional[float]:
    try:
        stat = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
        fields_after_comm = stat[stat.rfind(")") + 2 :].split()
        start_ticks = int(fields_after_comm[19])
        uptime_s = float(
            Path("/proc/uptime").read_text(encoding="utf-8").split()[0]
        )
        ticks_per_second = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
        return max(0.0, uptime_s - start_ticks / ticks_per_second)
    except (FileNotFoundError, ValueError, IndexError, OSError):
        return None


def phase_and_progress(log_lines: Sequence[str]) -> Tuple[str, bool, Optional[int]]:
    current_phase = "UNKNOWN"
    latest_start_id: Optional[int] = None
    ended_ids = set()
    for line in log_lines:
        phase_match = PHASE_PATTERN.search(line)
        if phase_match:
            current_phase = (
                f"PHASE {phase_match.group(1)} {phase_match.group(2)}"
            ).upper()
        start_match = RUN_START_PATTERN.search(line)
        if start_match:
            latest_start_id = int(start_match.group(1))
            phase_token = start_match.group(2)
            if phase_token and phase_token.startswith("phase"):
                current_phase = phase_token.replace("_", " ").upper()
            elif "PID STEP" in line:
                current_phase = "PHASE 3 PID STEP"
        end_match = RUN_END_PATTERN.search(line)
        if end_match:
            ended_ids.add(int(end_match.group(1)))
    in_progress = latest_start_id is not None and latest_start_id not in ended_ids
    return current_phase, in_progress, latest_start_id


def active_session(records: Sequence[Dict[str, Any]]) -> Optional[str]:
    if not records:
        return None
    value = records[-1].get("session_id")
    return str(value) if value else None


def average(records: Iterable[Dict[str, Any]], key: str) -> Optional[float]:
    values = [
        float(record[key])
        for record in records
        if isinstance(record.get(key), (float, int))
    ]
    return sum(values) / len(values) if values else None


def baseline_metrics(
    records: Sequence[Dict[str, Any]], session_id: Optional[str]
) -> Dict[str, Optional[float]]:
    baseline = [
        record
        for record in records
        if record.get("phase") == "phase0_baseline"
        and (session_id is None or record.get("session_id") == session_id)
        and record.get("lap_completed")
    ]
    return {
        key: average(baseline, key)
        for key in (
            "mean_path_error",
            "max_path_error",
            "max_abs_steering",
            "lap_time",
        )
    }


def best_candidate(
    best_path: Path, records: Sequence[Dict[str, Any]]
) -> Optional[Dict[str, Any]]:
    best_file = read_json(best_path)
    if best_file:
        configuration = best_file.get("configuration", {})
        metrics = best_file.get("metrics", {})
        if isinstance(configuration, dict) and isinstance(metrics, dict):
            return {
                "run_id": best_file.get("run_id"),
                "score": best_file.get("score"),
                **configuration,
                **metrics,
            }
    candidates = [
        record for record in records
        if record.get("lap_completed")
        and isinstance(record.get("score"), (float, int))
    ]
    return max(candidates, key=lambda record: float(record["score"])) \
        if candidates else None


def compact_config(record: Optional[Dict[str, Any]]) -> str:
    if not record:
        return "n/a"
    pid = "ON" if bool(record.get("pid_enabled")) else "OFF"
    return (
        f"minLA={fmt(record.get('min_lookahead'), 3)} "
        f"maxLA={fmt(record.get('max_lookahead'), 3)} "
        f"curvRef={fmt(record.get('curvature_reference'), 2)} "
        f"preview={fmt(record.get('preview_distance'), 2)} "
        f"maxLatAcc={fmt(record.get('max_lateral_acceleration'), 2)} "
        f"PID={pid}"
    )


def compact_metrics(record: Optional[Dict[str, Any]]) -> str:
    if not record:
        return "n/a"
    steering = degrees(record.get("max_abs_steering"))
    return (
        f"lap={fmt(record.get('lap_time'), 2)}s "
        f"meanErr={fmt(record.get('mean_path_error'), 4)}m "
        f"maxErr={fmt(record.get('max_path_error'), 4)}m "
        f"steer={fmt(steering, 2)}deg "
        f"sat={record.get('steering_saturation_count', 0)} "
        f"score={fmt(record.get('score'), 2)}"
    )


def improvement_line(
    baseline: Dict[str, Optional[float]], best: Optional[Dict[str, Any]]
) -> str:
    if not best:
        return "improvement (+ better): n/a"
    metrics = (
        ("meanErr", "mean_path_error"),
        ("maxErr", "max_path_error"),
        ("steer", "max_abs_steering"),
        ("lap", "lap_time"),
    )
    values = []
    for label, key in metrics:
        before = baseline.get(key)
        after = best.get(key)
        if not isinstance(before, (float, int)) or not isinstance(
            after, (float, int)
        ) or abs(float(before)) <= 1.0e-12:
            values.append(f"{label}=n/a")
            continue
        improvement = (float(before) - float(after)) / abs(float(before)) * 100.0
        values.append(f"{label}={improvement:+.2f}%")
    return "improvement (+ better): " + " ".join(values)


def changed_parameters(
    previous: Optional[Dict[str, Any]], current: Dict[str, Any]
) -> str:
    changes = []
    if previous is not None:
        for field in CONFIG_FIELDS:
            before = previous.get(field)
            after = current.get(field)
            if before != after:
                if field == "pid_enabled":
                    before = "ON" if bool(before) else "OFF"
                    after = "ON" if bool(after) else "OFF"
                else:
                    before = fmt(before, 3)
                    after = fmt(after, 3)
                changes.append(f"{PARAM_LABELS[field]} {before}->{after}")
    change_text = ", ".join(changes) if changes else "no config change"
    return f"run{current.get('run_id')}: {change_text}"


def recent_change_lines(records: Sequence[Dict[str, Any]]) -> List[str]:
    if not records:
        return ["  no completed records"]
    start = max(0, len(records) - 5)
    lines = []
    for index in range(start, len(records)):
        previous = records[index - 1] if index > 0 else None
        lines.append("  " + changed_parameters(previous, records[index]))
    return lines


def fmt(value: Any, digits: int) -> str:
    if not isinstance(value, (float, int)):
        return "n/a"
    number = float(value)
    if not math.isfinite(number):
        return "n/a"
    return f"{number:.{digits}f}"


def degrees(value: Any) -> Optional[float]:
    if not isinstance(value, (float, int)):
        return None
    return math.degrees(float(value))


def format_duration(seconds: Optional[float]) -> str:
    if seconds is None:
        return "n/a"
    total = int(seconds)
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def render_status(results_dir: Path, pid: int) -> Tuple[str, bool]:
    alive = is_autotune_alive(pid)
    records = read_jsonl(results_dir / "runs.jsonl")
    log_tail_for_state = tail_lines(results_dir / "autotune.log", 80)
    display_log_tail = log_tail_for_state[-6:]
    phase, in_progress, active_run = phase_and_progress(log_tail_for_state)
    latest = records[-1] if records else None
    best = best_candidate(results_dir / "best_so_far.json", records)
    session_id = active_session(records)
    baseline = baseline_metrics(records, session_id)
    successful_laps = sum(bool(record.get("lap_completed")) for record in records)
    failed_runs = sum(not bool(record.get("success")) for record in records)
    now = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")
    state = (
        f"RUN IN PROGRESS (run {active_run})"
        if alive and in_progress
        else ("IDLE BETWEEN RUNS" if alive else "AUTOTUNE STOPPED")
    )

    lines = [
        "=" * 50,
        "AUTOTUNE LIVE STATUS",
        "=" * 50,
        f"time: {now}",
        f"elapsed: {format_duration(process_elapsed_s(pid))}  "
        f"autotune_pid={pid} alive={'YES' if alive else 'NO'}",
        f"phase: {phase}  state: {state}",
        f"runs={len(records)} successful_laps={successful_laps} "
        f"failed_runs={failed_runs}",
        "latest: "
        + (
            f"run={latest.get('run_id')} complete={latest.get('lap_completed')} "
            + compact_config(latest)
            if latest
            else "n/a"
        ),
        "        " + compact_metrics(latest),
        "best:   "
        + (
            f"run={best.get('run_id')} {compact_config(best)}"
            if best
            else "n/a"
        ),
        "        "
        + (
            f"kp={fmt(best.get('kp'), 3)} ki={fmt(best.get('ki'), 3)} "
            f"kd={fmt(best.get('kd'), 3)} {compact_metrics(best)}"
            if best
            else "n/a"
        ),
        improvement_line(baseline, best),
        "recent parameter changes:",
        *recent_change_lines(records),
        "autotune.log tail:",
        *("  " + line for line in display_log_tail),
        "=" * 50,
    ]
    return "\n".join(lines), alive


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results-dir", type=Path, default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S)
    parser.add_argument("--autotune-pid", type=int)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args(argv)
    if not 15.0 <= args.interval <= 30.0:
        parser.error("--interval must be within 15..30 seconds")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    pid = args.autotune_pid or find_autotune_pid()
    if pid is None:
        print("AUTOTUNE STOPPED: no sim_control_autotune.py process found", flush=True)
        return 2
    while True:
        try:
            output, alive = render_status(args.results_dir, pid)
            print(output, flush=True)
        except Exception as error:
            print(
                f"AUTOTUNE WATCHER ERROR (autotune unaffected): {error}",
                file=sys.stderr,
                flush=True,
            )
            alive = is_autotune_alive(pid)
        if args.once:
            return 0 if alive else 2
        if not alive:
            return 2
        time.sleep(args.interval)


if __name__ == "__main__":
    raise SystemExit(main())

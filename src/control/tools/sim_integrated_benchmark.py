#!/usr/bin/env python3
"""Headless lap monitor for the production Planner + Control stack."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import statistics
import subprocess
import time
from urllib.request import Request, urlopen

from nav_msgs.msg import Odometry, Path as PathMessage
from rcl_interfaces.msg import Log
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64


ROOT = Path(__file__).resolve().parents[3]
API = 'http://localhost/sim/api'
PLAN_RE = re.compile(
    r'planning status=(\w+) time_ms=([0-9.]+) expanded_nodes=(\d+) path_points=(\d+)'
)


def request(endpoint: str, method: str = 'GET') -> dict:
    data = b'{}' if method == 'POST' else None
    req = Request(
        f'{API}/{endpoint}', data=data, method=method,
        headers={'Content-Type': 'application/json'},
    )
    with urlopen(req, timeout=2.0) as response:
        return json.loads(response.read() or b'{}')


def cache_matches(path: Path, commit: str, config_hash: str) -> bool:
    try:
        cached = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return False
    return (
        cached.get('commit') == commit
        and cached.get('config_sha256') == config_hash
    )


def percentile(values: list[float], ratio: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * ratio)]


def segment_distance(x: float, y: float, a: tuple[float, float], b: tuple[float, float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = (
        0.0 if length2 == 0.0
        else max(0.0, min(1.0, ((x - a[0]) * dx + (y - a[1]) * dy) / length2))
    )
    return math.hypot(x - a[0] - t * dx, y - a[1] - t * dy)


def nearest_index(
    points: list[tuple[float, float]], x: float, y: float, previous: int | None
) -> int:
    count = len(points) - 1
    candidates = range(count) if previous is None else (
        (previous + offset) % count for offset in range(-10, 16)
    )
    return min(candidates, key=lambda i: (points[i][0] - x) ** 2 + (points[i][1] - y) ** 2)


def path_metrics(message: PathMessage) -> dict | None:
    points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
    if len(points) < 3:
        return None
    distances = [math.dist(a, b) for a, b in zip(points, points[1:])]
    curvatures = []
    for a, b, c in zip(points, points[1:], points[2:]):
        ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
        denominator = ab * bc * ca
        cross = abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))
        curvatures.append(0.0 if denominator < 1e-12 else 2.0 * cross / denominator)
    changes = [abs(b - a) for a, b in zip(curvatures, curvatures[1:])]
    return {
        'length_m': sum(distances),
        'max_abs_curvature_inv_m': max(curvatures),
        'mean_abs_curvature_inv_m': statistics.fmean(curvatures),
        'curvature_squared_integral': sum(k * k * d for k, d in zip(curvatures, distances)),
        'mean_curvature_change_inv_m': statistics.fmean(changes) if changes else 0.0,
        'max_required_steering_rad': math.atan(0.18 * max(curvatures)),
    }


class Monitor(Node):
    def __init__(self) -> None:
        from interfaces.msg import SearchTree

        super().__init__('sim_integrated_benchmark')
        self.path_times: list[float] = []
        self.pose_times: list[float] = []
        self.steering: list[tuple[float, float]] = []
        self.speed_commands: list[float] = []
        self.actual_speeds: list[tuple[float, float]] = []
        self.plans: list[dict] = []
        self.paths: list[dict] = []
        self.empty_paths = 0
        self.tree_nodes: list[int] = []
        self.create_subscription(PathMessage, '/path', self.on_path, 10)
        self.create_subscription(
            Odometry, '/pose/calibration', self.on_pose, qos_profile_sensor_data
        )
        self.create_subscription(Float64, '/steering', self.on_steering, 10)
        self.create_subscription(
            Float64, '/speed',
            lambda msg: self.speed_commands.append(float(msg.data)), 10,
        )
        self.create_subscription(
            SearchTree, '/path_planning/debug/search_tree',
            lambda msg: self.tree_nodes.append(len(msg.x)),
            qos_profile_sensor_data,
        )
        self.create_subscription(Log, '/rosout', self.on_log, 100)
        self.gosign_pub = self.create_publisher(Bool, '/gosign', 10)
        self.stop_pub = self.create_publisher(Float64, '/speed', 10)
        self.steer_pub = self.create_publisher(Float64, '/steering', 10)

    def on_path(self, message: PathMessage) -> None:
        self.path_times.append(time.monotonic())
        metrics = path_metrics(message)
        if metrics is None:
            self.empty_paths += 1
        else:
            self.paths.append(metrics)

    def on_pose(self, message: Odometry) -> None:
        self.pose_times.append(time.monotonic())
        self.actual_speeds.append((time.monotonic(), float(message.twist.twist.linear.x)))

    def on_steering(self, message: Float64) -> None:
        self.steering.append((time.monotonic(), float(message.data)))

    def on_log(self, message: Log) -> None:
        match = PLAN_RE.search(message.msg)
        if match:
            self.plans.append({
                'status': match.group(1),
                'time_ms': float(match.group(2)),
                'expanded_nodes': int(match.group(3)),
                'path_points': int(match.group(4)),
            })


def rate(times: list[float]) -> float | None:
    return None if len(times) < 2 else (len(times) - 1) / (times[-1] - times[0])


def summarize(
    node: Monitor, samples: list[dict], elapsed: float, completed: bool, reason: str
) -> dict:
    errors = [row['lateral_error_m'] for row in samples]
    clearances = [row['boundary_clearance_m'] for row in samples]
    steer = node.steering
    steering_rates = [abs(b[1] - a[1]) / max(b[0] - a[0], 1e-6) for a, b in zip(steer, steer[1:])]
    variation = sum(abs(b[1] - a[1]) for a, b in zip(steer, steer[1:]))
    plan_times = [p['time_ms'] for p in node.plans]
    success_plans = [p for p in node.plans if p['status'] == 'success']
    return {
        'completed': completed,
        'termination_reason': reason,
        'lap_time_s': elapsed if completed else None,
        'sample_count': len(samples),
        'rms_lateral_error_m': (
            math.sqrt(statistics.fmean(e * e for e in errors)) if errors else None
        ),
        'max_lateral_error_m': max(errors, default=None),
        'minimum_boundary_clearance_m': min(clearances, default=None),
        'path_publish_hz': rate(node.path_times),
        'pose_publish_hz': rate(node.pose_times),
        'path_empty_count': node.empty_paths,
        'path_stale_gap_count': sum(
            b - a > 0.5 for a, b in zip(node.path_times, node.path_times[1:])
        ),
        'planning_attempts': len(node.plans),
        'planning_successes': len(success_plans),
        'planning_failures': len(node.plans) - len(success_plans),
        'planning_success_rate': len(success_plans) / len(node.plans) if node.plans else None,
        'planning_time_mean_ms': statistics.fmean(plan_times) if plan_times else None,
        'planning_time_max_ms': max(plan_times, default=None),
        'expanded_nodes_mean': (
            statistics.fmean(p['expanded_nodes'] for p in success_plans)
            if success_plans else None
        ),
        'expanded_nodes_max': max((p['expanded_nodes'] for p in success_plans), default=None),
        'search_tree_nodes_mean': statistics.fmean(node.tree_nodes) if node.tree_nodes else None,
        'path_length_mean_m': (
            statistics.fmean(p['length_m'] for p in node.paths) if node.paths else None
        ),
        'path_max_abs_curvature_inv_m': max(
            (p['max_abs_curvature_inv_m'] for p in node.paths), default=None
        ),
        'path_mean_abs_curvature_inv_m': (
            statistics.fmean(p['mean_abs_curvature_inv_m'] for p in node.paths)
            if node.paths else None
        ),
        'path_curvature_squared_integral_mean': (
            statistics.fmean(p['curvature_squared_integral'] for p in node.paths)
            if node.paths else None
        ),
        'path_mean_curvature_change_inv_m': (
            statistics.fmean(p['mean_curvature_change_inv_m'] for p in node.paths)
            if node.paths else None
        ),
        'path_max_required_steering_rad': max(
            (p['max_required_steering_rad'] for p in node.paths), default=None
        ),
        'path_exceeds_steering_limit': any(
            p['max_required_steering_rad'] > 0.3491 + 1e-4 for p in node.paths
        ),
        'steering_max_abs_rad': max((abs(v) for _, v in steer), default=None),
        'steering_saturation_samples': sum(abs(v) >= 0.3490 for _, v in steer),
        'steering_total_variation_rad': variation,
        'steering_rate_p95_rad_s': percentile(steering_rates, 0.95),
        'steering_rate_max_rad_s': max(steering_rates, default=None),
        'speed_command_mean_m_s': (
            statistics.fmean(node.speed_commands) if node.speed_commands else None
        ),
        'speed_command_min_m_s': min(node.speed_commands, default=None),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', required=True)
    parser.add_argument(
        '--output-dir', type=Path,
        default=ROOT / 'artifacts/integrated_optimization_2026-08-25/runs',
    )
    parser.add_argument('--timeout', type=float, default=150.0)
    parser.add_argument('--launch-timeout', type=float, default=180.0)
    parser.add_argument('--force', action='store_true')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = args.output_dir / f'{args.label}.json'
    config_files = [
        ROOT / 'src/control/config/control.yaml',
        ROOT / 'src/path_planning/config/path_planning.yaml',
    ]
    config_hash = hashlib.sha256(b''.join(p.read_bytes() for p in config_files)).hexdigest()
    commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()
    if summary_path.exists() and not args.force:
        if cache_matches(summary_path, commit, config_hash):
            print(f'cached: {summary_path}')
            return 0
        print(f'cache invalidated: {summary_path}')

    world = request('world')
    track = world['track']
    route = [tuple(map(float, p)) for p in track['route']['waypoints']]
    inner = [tuple(map(float, p)) for p in track['route']['inner']]
    outer = [tuple(map(float, p)) for p in track['route']['outer']]

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Monitor()
    samples: list[dict] = []
    previous_index = None
    advance = 0
    completed = False
    reason = 'timeout'
    process = None
    log_file = None
    started = None
    ready_at = None
    elapsed = 0.0
    try:
        request('reset', 'POST')
        log_file = (args.output_dir / f'{args.label}.log').open('w')
        process = subprocess.Popen(
            [str(ROOT / 'run.sh')], cwd=ROOT, stdout=log_file,
            stderr=subprocess.STDOUT, start_new_session=True,
        )
        launch_deadline = time.monotonic() + args.launch_timeout
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            ready = bool(node.paths and node.pose_times)
            node.gosign_pub.publish(Bool(data=ready))
            if ready and ready_at is None:
                ready_at = now
            if not ready and now >= launch_deadline:
                reason = 'launch_timeout'
                break
            if ready_at is not None and started is None and now - ready_at >= 30.0:
                reason = 'departure_timeout'
                break
            if (
                started is None and ready and node.actual_speeds
                and abs(node.actual_speeds[-1][1]) > 0.02
            ):
                started = now
            if started is not None and now >= started + args.timeout:
                reason = 'lap_timeout'
                break
            if process.poll() is not None:
                reason = 'run_process_stopped'
                break
            if not ready:
                continue
            state = request('state')
            vehicle = state.get('vehicle')
            if not vehicle:
                continue
            x, y = float(vehicle['x']), float(vehicle['y'])
            index = nearest_index(route, x, y, previous_index)
            if previous_index is not None:
                delta = index - previous_index
                count = len(route) - 1
                if delta > count / 2:
                    delta -= count
                elif delta < -count / 2:
                    delta += count
                advance += delta
            previous_index = index
            center_error = segment_distance(
                x, y, route[index], route[(index + 1) % (len(route) - 1)]
            )
            boundary = min(
                segment_distance(x, y, inner[index], inner[(index + 1) % (len(inner) - 1)]),
                segment_distance(x, y, outer[index], outer[(index + 1) % (len(outer) - 1)]),
            ) - 0.10
            samples.append({
                'elapsed_s': 0.0 if started is None else time.monotonic() - started,
                'x_m': x, 'y_m': y, 'route_index': index,
                'route_advance': advance,
                'lateral_error_m': center_error,
                'boundary_clearance_m': boundary,
            })
            count = len(route) - 1
            if advance >= count - 15 and index <= 15:
                completed, reason = True, 'lap_complete'
                break
    finally:
        elapsed = 0.0 if started is None else time.monotonic() - started
        for _ in range(5):
            node.gosign_pub.publish(Bool(data=False))
            node.stop_pub.publish(Float64(data=0.0))
            node.steer_pub.publish(Float64(data=0.0))
            rclpy.spin_once(node, timeout_sec=0.05)
        try:
            request('evaluation/stop', 'POST')
        except OSError:
            pass
        try:
            if process is not None and process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.0)
        finally:
            if log_file is not None:
                log_file.close()
            node.destroy_node()
            rclpy.shutdown()

    summary = summarize(node, samples, elapsed, completed, reason)
    summary.update({
        'label': args.label,
        'commit': commit,
        'config_sha256': config_hash,
        'world_id': world.get('world_id'),
        'world_rev': world.get('rev'),
    })
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    with (args.output_dir / f'{args.label}.csv').open('w', newline='') as output:
        writer = csv.DictWriter(output, fieldnames=samples[0].keys() if samples else ['elapsed_s'])
        writer.writeheader()
        writer.writerows(samples)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if completed else 2


if __name__ == '__main__':
    raise SystemExit(main())

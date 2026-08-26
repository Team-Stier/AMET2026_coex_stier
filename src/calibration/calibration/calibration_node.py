from __future__ import annotations

import copy
import math
import time

from geometry_msgs.msg import TwistWithCovariance
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from calibration.wall_fitter import (
    FitResult,
    RectangleWallFitter,
    ego_pose_from_lidar,
    lidar_pose_from_ego,
)


def propagate_pose_with_relative_motion(
    calibrated_anchor: tuple[float, float, float],
    prior_anchor: tuple[float, float, float],
    current_prior: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Apply only a prior's relative SE(2) motion to a calibrated map pose."""
    anchor_x, anchor_y, anchor_yaw = prior_anchor
    current_x, current_y, current_yaw = current_prior
    calibrated_x, calibrated_y, calibrated_yaw = calibrated_anchor

    delta_x = current_x - anchor_x
    delta_y = current_y - anchor_y
    local_x = math.cos(anchor_yaw) * delta_x + math.sin(anchor_yaw) * delta_y
    local_y = -math.sin(anchor_yaw) * delta_x + math.cos(anchor_yaw) * delta_y
    predicted_x = (
        calibrated_x
        + math.cos(calibrated_yaw) * local_x
        - math.sin(calibrated_yaw) * local_y
    )
    predicted_y = (
        calibrated_y
        + math.sin(calibrated_yaw) * local_x
        + math.cos(calibrated_yaw) * local_y
    )
    predicted_yaw = math.atan2(
        math.sin(calibrated_yaw + current_yaw - anchor_yaw),
        math.cos(calibrated_yaw + current_yaw - anchor_yaw),
    )
    return predicted_x, predicted_y, predicted_yaw


def _pose_correction(
    reference_pose: tuple[float, float, float],
    candidate_pose: tuple[float, float, float],
) -> tuple[float, float]:
    position_m = math.hypot(
        candidate_pose[0] - reference_pose[0],
        candidate_pose[1] - reference_pose[1],
    )
    yaw_rad = abs(
        math.atan2(
            math.sin(candidate_pose[2] - reference_pose[2]),
            math.cos(candidate_pose[2] - reference_pose[2]),
        )
    )
    return position_m, yaw_rad


def prior_motion_is_continuous(
    previous_pose: tuple[float, float, float],
    current_pose: tuple[float, float, float],
    *,
    elapsed_sec: float,
    maximum_linear_speed_m_s: float,
    maximum_yaw_rate_rad_s: float,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> bool:
    """Check a prior increment against elapsed-time-scaled physical bounds."""
    if not math.isfinite(elapsed_sec) or elapsed_sec <= 0.0:
        return False
    position_m, yaw_rad = _pose_correction(previous_pose, current_pose)
    return (
        position_m
        <= position_tolerance_m + maximum_linear_speed_m_s * elapsed_sec
        and yaw_rad
        <= yaw_tolerance_rad + maximum_yaw_rate_rad_s * elapsed_sec
    )


def fit_with_bounded_pose_recovery(
    fitter: RectangleWallFitter,
    points: np.ndarray,
    reference_pose: tuple[float, float, float],
    *,
    xy_offsets_m: tuple[float, ...],
    yaw_offsets_rad: tuple[float, ...],
    maximum_rms_error_m: float,
    maximum_position_correction_m: float,
    maximum_yaw_correction_rad: float,
    rms_tie_tolerance_m: float,
    ambiguity_position_m: float,
    ambiguity_yaw_rad: float,
) -> FitResult | None:
    """Search a bounded pose grid and reject distinct near-equal solutions."""
    candidates: list[tuple[FitResult, int, float, float, int]] = []
    reference_x, reference_y, reference_yaw = reference_pose
    seed_index = 0
    for x_offset in xy_offsets_m:
        for y_offset in xy_offsets_m:
            for yaw_offset in yaw_offsets_rad:
                seed = (
                    reference_x + x_offset,
                    reference_y + y_offset,
                    math.atan2(
                        math.sin(reference_yaw + yaw_offset),
                        math.cos(reference_yaw + yaw_offset),
                    ),
                )
                result = fitter.fit(points, seed)
                current_seed_index = seed_index
                seed_index += 1
                if result is None or result.rms_error_m > maximum_rms_error_m:
                    continue

                position_m, yaw_rad = _pose_correction(
                    reference_pose, result.pose
                )
                if (
                    position_m > maximum_position_correction_m
                    or yaw_rad > maximum_yaw_correction_rad
                ):
                    continue
                qualified_walls = sum(
                    count >= fitter.minimum_matches_per_wall
                    for count in result.wall_match_counts
                )
                candidates.append(
                    (
                        result,
                        qualified_walls,
                        position_m,
                        yaw_rad,
                        current_seed_index,
                    )
                )

    if not candidates:
        return None

    maximum_qualified_walls = max(candidate[1] for candidate in candidates)
    observable = [
        candidate
        for candidate in candidates
        if candidate[1] == maximum_qualified_walls
    ]
    minimum_rms_error_m = min(candidate[0].rms_error_m for candidate in observable)
    near_best = [
        candidate
        for candidate in observable
        if candidate[0].rms_error_m
        <= minimum_rms_error_m + rms_tie_tolerance_m
    ]
    selected = min(
        near_best,
        key=lambda candidate: (
            candidate[3],
            candidate[2],
            candidate[0].rms_error_m,
            candidate[4],
        ),
    )
    for candidate in near_best:
        position_m, yaw_rad = _pose_correction(
            selected[0].pose, candidate[0].pose
        )
        if position_m > ambiguity_position_m or yaw_rad > ambiguity_yaw_rad:
            return None
    return selected[0]


def fit_with_yaw_recovery(
    fitter: RectangleWallFitter,
    points: np.ndarray,
    reference_pose: tuple[float, float, float],
    yaw_offsets_rad: tuple[float, ...],
    maximum_rms_error_m: float,
    maximum_position_correction_m: float,
    maximum_yaw_correction_rad: float,
    rms_tie_tolerance_m: float,
) -> FitResult | None:
    """Retry a failed fit from nearby yaw seeds without widening pose limits."""
    candidates: list[
        tuple[FitResult, int, float, float, int]
    ] = []
    reference_x, reference_y, reference_yaw = reference_pose
    for offset_index, yaw_offset in enumerate(yaw_offsets_rad):
        seed = (
            reference_x,
            reference_y,
            math.atan2(
                math.sin(reference_yaw + yaw_offset),
                math.cos(reference_yaw + yaw_offset),
            ),
        )
        result = fitter.fit(points, seed)
        if result is None or result.rms_error_m > maximum_rms_error_m:
            continue

        position_correction_m = math.hypot(
            result.pose[0] - reference_x,
            result.pose[1] - reference_y,
        )
        yaw_correction_rad = abs(
            math.atan2(
                math.sin(result.pose[2] - reference_yaw),
                math.cos(result.pose[2] - reference_yaw),
            )
        )
        if (
            position_correction_m > maximum_position_correction_m
            or yaw_correction_rad > maximum_yaw_correction_rad
        ):
            continue

        qualified_wall_count = sum(
            count >= fitter.minimum_matches_per_wall
            for count in result.wall_match_counts
        )
        candidates.append(
            (
                result,
                qualified_wall_count,
                position_correction_m,
                yaw_correction_rad,
                offset_index,
            )
        )

    if not candidates:
        return None

    maximum_qualified_walls = max(candidate[1] for candidate in candidates)
    observable_candidates = [
        candidate
        for candidate in candidates
        if candidate[1] == maximum_qualified_walls
    ]
    minimum_rms_error_m = min(
        candidate[0].rms_error_m for candidate in observable_candidates
    )
    near_best_candidates = [
        candidate
        for candidate in observable_candidates
        if candidate[0].rms_error_m
        <= minimum_rms_error_m + rms_tie_tolerance_m
    ]
    selected = min(
        near_best_candidates,
        key=lambda candidate: (
            candidate[3],
            candidate[2],
            candidate[0].rms_error_m,
            candidate[4],
        ),
    )
    return selected[0]


class CalibrationNode(Node):
    """Publish a map pose by fitting /scan returns to the rectangular walls."""

    def __init__(self) -> None:
        super().__init__("calibration_node")
        self._declare_parameters()

        self._lidar_frame = str(self.get_parameter("lidar_frame").value)
        self._lidar_offset_x_m = float(
            self.get_parameter("lidar_offset_x_m").value
        )
        self._scan_stride = max(1, int(self.get_parameter("scan_stride").value))
        self._maximum_rms_error_m = float(
            self.get_parameter("maximum_rms_error_m").value
        )
        self._fallback_position_variance_m2 = float(
            self.get_parameter("fallback_position_variance_m2").value
        )
        self._fallback_yaw_variance_rad2 = float(
            self.get_parameter("fallback_yaw_variance_rad2").value
        )
        maximum_prior_age_sec = float(
            self.get_parameter("maximum_prior_age_sec").value
        )
        maximum_prior_gap_sec = float(
            self.get_parameter("maximum_prior_gap_sec").value
        )
        scan_prior_sync_timeout_sec = float(
            self.get_parameter("scan_prior_sync_timeout_sec").value
        )
        self._scan_prior_buffer_size = int(
            self.get_parameter("scan_prior_buffer_size").value
        )
        self._maximum_prior_linear_speed_m_s = float(
            self.get_parameter("maximum_prior_linear_speed_m_s").value
        )
        self._maximum_prior_yaw_rate_rad_s = float(
            self.get_parameter("maximum_prior_yaw_rate_rad_s").value
        )
        self._prior_position_jump_tolerance_m = float(
            self.get_parameter("prior_position_jump_tolerance_m").value
        )
        self._prior_yaw_jump_tolerance_rad = float(
            self.get_parameter("prior_yaw_jump_tolerance_rad").value
        )
        maximum_fallback_duration_sec = float(
            self.get_parameter("maximum_fallback_duration_sec").value
        )
        initial_ego_pose = (
            float(self.get_parameter("initial_pose_x_m").value),
            float(self.get_parameter("initial_pose_y_m").value),
            float(self.get_parameter("initial_pose_yaw_rad").value),
        )
        if (
            not math.isfinite(self._lidar_offset_x_m)
            or not math.isfinite(self._maximum_rms_error_m)
            or self._maximum_rms_error_m <= 0.0
            or not math.isfinite(maximum_prior_age_sec)
            or maximum_prior_age_sec <= 0.0
            or not math.isfinite(maximum_prior_gap_sec)
            or maximum_prior_gap_sec <= maximum_prior_age_sec
            or not math.isfinite(scan_prior_sync_timeout_sec)
            or scan_prior_sync_timeout_sec <= 0.0
            or scan_prior_sync_timeout_sec > maximum_prior_age_sec
            or self._scan_prior_buffer_size < 2
            or not math.isfinite(self._maximum_prior_linear_speed_m_s)
            or self._maximum_prior_linear_speed_m_s <= 0.0
            or not math.isfinite(self._maximum_prior_yaw_rate_rad_s)
            or self._maximum_prior_yaw_rate_rad_s <= 0.0
            or not math.isfinite(self._prior_position_jump_tolerance_m)
            or self._prior_position_jump_tolerance_m < 0.0
            or not math.isfinite(self._prior_yaw_jump_tolerance_rad)
            or self._prior_yaw_jump_tolerance_rad < 0.0
            or not math.isfinite(maximum_fallback_duration_sec)
            or maximum_fallback_duration_sec <= 0.0
            or not math.isfinite(self._fallback_position_variance_m2)
            or self._fallback_position_variance_m2 <= 0.0
            or not math.isfinite(self._fallback_yaw_variance_rad2)
            or self._fallback_yaw_variance_rad2 <= 0.0
            or not all(math.isfinite(value) for value in initial_ego_pose)
        ):
            raise ValueError("pose parameters must be finite and thresholds positive")
        self._maximum_prior_age_ns = int(maximum_prior_age_sec * 1.0e9)
        self._maximum_prior_gap_ns = int(maximum_prior_gap_sec * 1.0e9)
        self._scan_prior_sync_timeout_ns = int(
            scan_prior_sync_timeout_sec * 1.0e9
        )
        self._maximum_fallback_duration_ns = int(
            maximum_fallback_duration_sec * 1.0e9
        )
        self._initial_pose = lidar_pose_from_ego(
            initial_ego_pose, self._lidar_offset_x_m
        )
        self._pose = self._initial_pose
        self._tracking_initialized = False
        self._last_stamp_ns: int | None = None
        self._latest_prior_pose: tuple[float, float, float] | None = None
        self._latest_prior_stamp_ns: int | None = None
        self._latest_prior_twist: TwistWithCovariance | None = None
        self._latest_prior_twist_stamp_ns: int | None = None
        self._prior_pose_buffer: dict[int, tuple[float, float, float]] = {}
        self._prior_twist_buffer: dict[int, TwistWithCovariance | None] = {}
        self._pending_scans: dict[int, tuple[LaserScan, int]] = {}
        self._tracking_pose_anchor: tuple[float, float, float] | None = None
        self._tracking_prior_anchor: tuple[float, float, float] | None = None
        self._last_fit_stamp_ns: int | None = None
        self._consecutive_fit_failures = 0
        self._pending_recovery_pose: tuple[float, float, float] | None = None
        self._pending_recovery_prior: tuple[float, float, float] | None = None
        self._pending_recovery_stamp_ns: int | None = None
        self._pending_recovery_count = 0
        self._last_fit_mode: str | None = None

        wall_bounds = (
            float(self.get_parameter("wall_minimum_x_m").value),
            float(self.get_parameter("wall_maximum_x_m").value),
            float(self.get_parameter("wall_minimum_y_m").value),
            float(self.get_parameter("wall_maximum_y_m").value),
        )
        minimum_walls = int(self.get_parameter("minimum_walls").value)
        self._two_wall_maximum_rms_error_m = float(
            self.get_parameter("two_wall_maximum_rms_error_m").value
        )
        self._two_wall_maximum_position_step_m = float(
            self.get_parameter("two_wall_maximum_position_step_m").value
        )
        self._two_wall_maximum_yaw_step_rad = float(
            self.get_parameter("two_wall_maximum_yaw_step_rad").value
        )
        self._yaw_recovery_enabled = bool(
            self.get_parameter("yaw_recovery_enabled").value
        )
        self._yaw_recovery_offsets_rad = tuple(
            float(value)
            for value in self.get_parameter("yaw_recovery_offsets_rad").value
        )
        self._yaw_recovery_rms_tie_tolerance_m = float(
            self.get_parameter("yaw_recovery_rms_tie_tolerance_m").value
        )
        self._fallback_recovery_enabled = bool(
            self.get_parameter("fallback_recovery_enabled").value
        )
        self._fallback_recovery_trigger_frames = int(
            self.get_parameter("fallback_recovery_trigger_frames").value
        )
        self._recovery_confirmation_frames = int(
            self.get_parameter("fallback_recovery_confirmation_frames").value
        )
        self._fallback_recovery_xy_offsets_m = tuple(
            float(value)
            for value in self.get_parameter(
                "fallback_recovery_xy_offsets_m"
            ).value
        )
        self._fallback_recovery_yaw_offsets_rad = tuple(
            float(value)
            for value in self.get_parameter(
                "fallback_recovery_yaw_offsets_rad"
            ).value
        )
        self._fallback_recovery_maximum_position_correction_m = float(
            self.get_parameter(
                "fallback_recovery_maximum_position_correction_m"
            ).value
        )
        self._fallback_recovery_maximum_yaw_correction_rad = float(
            self.get_parameter(
                "fallback_recovery_maximum_yaw_correction_rad"
            ).value
        )
        self._recovery_ambiguity_position_m = float(
            self.get_parameter("fallback_recovery_ambiguity_position_m").value
        )
        self._recovery_ambiguity_yaw_rad = float(
            self.get_parameter("fallback_recovery_ambiguity_yaw_rad").value
        )
        self._recovery_consistency_position_m = float(
            self.get_parameter("fallback_recovery_consistency_position_m").value
        )
        self._recovery_consistency_yaw_rad = float(
            self.get_parameter("fallback_recovery_consistency_yaw_rad").value
        )
        if minimum_walls < 2 or minimum_walls > 4:
            raise ValueError("minimum_walls must be between 2 and 4")
        if (
            not math.isfinite(self._two_wall_maximum_rms_error_m)
            or self._two_wall_maximum_rms_error_m <= 0.0
            or not math.isfinite(self._two_wall_maximum_position_step_m)
            or self._two_wall_maximum_position_step_m <= 0.0
            or not math.isfinite(self._two_wall_maximum_yaw_step_rad)
            or self._two_wall_maximum_yaw_step_rad <= 0.0
            or not math.isfinite(self._yaw_recovery_rms_tie_tolerance_m)
            or self._yaw_recovery_rms_tie_tolerance_m < 0.0
            or self._fallback_recovery_trigger_frames < 1
            or self._recovery_confirmation_frames < 1
            or not self._fallback_recovery_xy_offsets_m
            or not self._fallback_recovery_yaw_offsets_rad
            or any(
                not math.isfinite(value)
                for value in self._fallback_recovery_xy_offsets_m
            )
            or any(
                not math.isfinite(value)
                or abs(value) > math.pi
                for value in self._fallback_recovery_yaw_offsets_rad
            )
            or not math.isfinite(
                self._fallback_recovery_maximum_position_correction_m
            )
            or self._fallback_recovery_maximum_position_correction_m <= 0.0
            or not math.isfinite(
                self._fallback_recovery_maximum_yaw_correction_rad
            )
            or self._fallback_recovery_maximum_yaw_correction_rad <= 0.0
            or not math.isfinite(self._recovery_ambiguity_position_m)
            or self._recovery_ambiguity_position_m <= 0.0
            or not math.isfinite(self._recovery_ambiguity_yaw_rad)
            or self._recovery_ambiguity_yaw_rad <= 0.0
            or not math.isfinite(self._recovery_consistency_position_m)
            or self._recovery_consistency_position_m <= 0.0
            or not math.isfinite(self._recovery_consistency_yaw_rad)
            or self._recovery_consistency_yaw_rad <= 0.0
            or (
                self._yaw_recovery_enabled
                and not self._yaw_recovery_offsets_rad
            )
            or any(
                not math.isfinite(value)
                or abs(value) < 1.0e-9
                or abs(value) > math.pi
                for value in self._yaw_recovery_offsets_rad
            )
        ):
            raise ValueError("two-wall and yaw-recovery parameters are invalid")
        self._fitter = RectangleWallFitter(
            wall_bounds,
            maximum_match_distance_m=float(
                self.get_parameter("maximum_match_distance_m").value
            ),
            minimum_matches=int(self.get_parameter("minimum_matches").value),
            minimum_walls=max(3, minimum_walls),
            minimum_matches_per_wall=int(
                self.get_parameter("minimum_matches_per_wall").value
            ),
            maximum_position_step_m=float(
                self.get_parameter("maximum_position_step_m").value
            ),
            maximum_yaw_step_rad=float(
                self.get_parameter("maximum_yaw_step_rad").value
            ),
        )
        self._two_wall_fitter: RectangleWallFitter | None = None
        if minimum_walls == 2:
            self._two_wall_fitter = RectangleWallFitter(
                wall_bounds,
                maximum_match_distance_m=float(
                    self.get_parameter("maximum_match_distance_m").value
                ),
                minimum_matches=int(self.get_parameter("minimum_matches").value),
                minimum_walls=2,
                minimum_matches_per_wall=int(
                    self.get_parameter("two_wall_minimum_matches_per_wall").value
                ),
                maximum_position_step_m=float(
                    self._two_wall_maximum_position_step_m
                ),
                maximum_yaw_step_rad=float(
                    self._two_wall_maximum_yaw_step_rad
                ),
            )

        self._publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self._subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter("scan_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._prior_subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter("prior_topic").value),
            self._on_prior,
            qos_profile_sensor_data,
        )
        self._sync_timer = self.create_timer(
            min(scan_prior_sync_timeout_sec / 2.0, 0.01),
            self._drain_pending_scans,
        )
        self.get_logger().info(
            "rectangle-wall calibration ready: /scan + /pose prior -> "
            "/pose/calibration; "
            f"initial ego pose={initial_ego_pose}, minimum walls={minimum_walls}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("prior_topic", "/pose")
        self.declare_parameter("output_topic", "/pose/calibration")
        self.declare_parameter("lidar_frame", "lidar_link")
        self.declare_parameter("lidar_offset_x_m", -0.027)
        self.declare_parameter("wall_minimum_x_m", 0.0)
        self.declare_parameter("wall_maximum_x_m", 12.0)
        self.declare_parameter("wall_minimum_y_m", 0.0)
        self.declare_parameter("wall_maximum_y_m", 7.0)
        # A rectangle has symmetric absolute poses. The first accepted wall fit
        # always starts from this seed; later fits keep that solution continuous.
        # First center point in rddf/rddf_real.csv, facing map -Y.
        self.declare_parameter("initial_pose_x_m", 1.400001)
        self.declare_parameter("initial_pose_y_m", 3.394607)
        self.declare_parameter("initial_pose_yaw_rad", -math.pi / 2.0)
        self.declare_parameter("scan_stride", 2)
        self.declare_parameter("maximum_match_distance_m", 0.30)
        self.declare_parameter("minimum_matches", 50)
        self.declare_parameter("minimum_walls", 2)
        self.declare_parameter("minimum_matches_per_wall", 8)
        self.declare_parameter("maximum_position_step_m", 0.45)
        self.declare_parameter("maximum_yaw_step_rad", 0.35)
        self.declare_parameter("maximum_rms_error_m", 0.10)
        # Fallback is deliberately less certain than a wall fit. These values
        # are exposed even though the current planner does not consume covariance.
        self.declare_parameter("fallback_position_variance_m2", 0.04)
        self.declare_parameter("fallback_yaw_variance_rad2", 0.030461741978670857)
        self.declare_parameter("two_wall_minimum_matches_per_wall", 12)
        self.declare_parameter("two_wall_maximum_position_step_m", 0.25)
        self.declare_parameter("two_wall_maximum_yaw_step_rad", 0.15)
        self.declare_parameter("two_wall_maximum_rms_error_m", 0.08)
        self.declare_parameter("yaw_recovery_enabled", True)
        self.declare_parameter(
            "yaw_recovery_offsets_rad", [-0.20, -0.10, 0.10, 0.20]
        )
        self.declare_parameter("yaw_recovery_rms_tie_tolerance_m", 0.005)
        self.declare_parameter("maximum_prior_age_sec", 0.25)
        self.declare_parameter("maximum_prior_gap_sec", 1.0)
        self.declare_parameter("scan_prior_sync_timeout_sec", 0.06)
        self.declare_parameter("scan_prior_buffer_size", 32)
        # /pose comes from laser odometry. Reject a discontinuity, but preserve
        # physically possible motion up to the controller's 3 m/s speed cap.
        self.declare_parameter("maximum_prior_linear_speed_m_s", 3.0)
        self.declare_parameter("maximum_prior_yaw_rate_rad_s", 3.0)
        self.declare_parameter("prior_position_jump_tolerance_m", 0.15)
        self.declare_parameter("prior_yaw_jump_tolerance_rad", 0.05)
        # A normal fit remains strict. Drift recovery searches nearby seeds and
        # becomes an anchor only after motion-consistent consecutive scans.
        self.declare_parameter("fallback_recovery_enabled", True)
        self.declare_parameter("fallback_recovery_trigger_frames", 3)
        self.declare_parameter("fallback_recovery_confirmation_frames", 5)
        self.declare_parameter(
            "fallback_recovery_xy_offsets_m", [-0.20, 0.0, 0.20]
        )
        self.declare_parameter(
            "fallback_recovery_yaw_offsets_rad", [-0.15, 0.0, 0.15]
        )
        self.declare_parameter(
            "fallback_recovery_maximum_position_correction_m", 0.40
        )
        self.declare_parameter(
            "fallback_recovery_maximum_yaw_correction_rad", 0.30
        )
        self.declare_parameter("fallback_recovery_ambiguity_position_m", 0.10)
        self.declare_parameter("fallback_recovery_ambiguity_yaw_rad", 0.10)
        self.declare_parameter("fallback_recovery_consistency_position_m", 0.30)
        self.declare_parameter("fallback_recovery_consistency_yaw_rad", 0.20)
        # Downstream ignores pose covariance, so a long uncorrected fallback
        # must stop publishing and let its existing pose watchdog stop control.
        self.declare_parameter("maximum_fallback_duration_sec", 1.0)

    def _prior_pose_for_stamp(
        self, stamp_ns: int
    ) -> tuple[float, float, float] | None:
        buffer = getattr(self, "_prior_pose_buffer", None)
        if buffer is not None and stamp_ns in buffer:
            return buffer[stamp_ns]
        if getattr(self, "_latest_prior_stamp_ns", None) == stamp_ns:
            return self._latest_prior_pose
        return None

    def _prior_twist_for_stamp(
        self, stamp_ns: int
    ) -> TwistWithCovariance | None:
        buffer = getattr(self, "_prior_twist_buffer", None)
        if buffer is not None and stamp_ns in buffer:
            return buffer[stamp_ns]
        if getattr(self, "_latest_prior_twist_stamp_ns", None) == stamp_ns:
            return self._latest_prior_twist
        return None

    def _prune_prior_buffers(self, newest_stamp_ns: int) -> None:
        cutoff_ns = newest_stamp_ns - self._maximum_prior_age_ns
        for buffer in (self._prior_pose_buffer, self._prior_twist_buffer):
            for stamp_ns in sorted(buffer):
                if (
                    stamp_ns >= cutoff_ns
                    and len(buffer) <= self._scan_prior_buffer_size
                ):
                    break
                buffer.pop(stamp_ns, None)

    def _drain_pending_scans(self, force: bool = False) -> None:
        pending_scans = getattr(self, "_pending_scans", None)
        if not pending_scans:
            return
        now_ns = time.monotonic_ns()
        while pending_scans:
            stamp_ns = min(pending_scans)
            scan, arrival_ns = pending_scans[stamp_ns]
            synchronized = (
                CalibrationNode._prior_pose_for_stamp(self, stamp_ns) is not None
            )
            expired = now_ns - arrival_ns >= self._scan_prior_sync_timeout_ns
            if not synchronized and not expired and not force:
                break
            pending_scans.pop(stamp_ns)
            CalibrationNode._process_scan(self, scan)

    def _on_prior(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        quaternion = np.asarray(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quaternion))
        if (
            message.header.frame_id != "map"
            or not math.isfinite(position.x)
            or not math.isfinite(position.y)
            or not math.isfinite(norm)
            or norm < 1.0e-9
        ):
            return
        qx, qy, qz, qw = quaternion / norm
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        current_prior_pose = lidar_pose_from_ego(
            (float(position.x), float(position.y), yaw), self._lidar_offset_x_m
        )
        prior_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        previous_prior_pose = getattr(self, "_latest_prior_pose", None)
        previous_prior_stamp_ns = getattr(self, "_latest_prior_stamp_ns", None)
        if (
            previous_prior_stamp_ns is not None
            and prior_stamp_ns <= previous_prior_stamp_ns
        ):
            self.get_logger().warning(
                "out-of-order /pose prior ignored",
                throttle_duration_sec=2.0,
            )
            return

        prior_discontinuity = False
        if previous_prior_pose is not None and previous_prior_stamp_ns is not None:
            elapsed_ns = prior_stamp_ns - previous_prior_stamp_ns
            if elapsed_ns > self._maximum_prior_gap_ns:
                # Motion during a long input gap is unknown. Do not reconnect
                # fallback until a wall fit establishes a synchronized anchor.
                self._tracking_pose_anchor = None
                self._tracking_prior_anchor = None
                CalibrationNode._clear_pending_recovery(self)
                prior_discontinuity = True
                self.get_logger().warning(
                    "/pose prior gap invalidated odometry fallback anchor",
                    throttle_duration_sec=2.0,
                )
            elif not prior_motion_is_continuous(
                previous_prior_pose,
                current_prior_pose,
                elapsed_sec=elapsed_ns * 1.0e-9,
                maximum_linear_speed_m_s=self._maximum_prior_linear_speed_m_s,
                maximum_yaw_rate_rad_s=self._maximum_prior_yaw_rate_rad_s,
                position_tolerance_m=self._prior_position_jump_tolerance_m,
                yaw_tolerance_rad=self._prior_yaw_jump_tolerance_rad,
            ):
                # Absorb an odometry coordinate jump without teleporting the
                # calibrated output. Future continuous increments start here.
                if (
                    self._tracking_initialized
                    and self._tracking_pose_anchor is not None
                    and self._tracking_prior_anchor is not None
                ):
                    self._tracking_pose_anchor = self._pose
                    self._tracking_prior_anchor = current_prior_pose
                CalibrationNode._clear_pending_recovery(self)
                prior_discontinuity = True
                self.get_logger().warning(
                    "/pose prior jump absorbed by rebasing fallback anchor",
                    throttle_duration_sec=2.0,
                )

        self._latest_prior_pose = current_prior_pose
        self._latest_prior_stamp_ns = prior_stamp_ns

        twist = message.twist.twist
        twist_values = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        forwarded_twist: TwistWithCovariance | None = None
        if not prior_discontinuity and all(
            math.isfinite(value) for value in twist_values
        ):
            forwarded_twist = copy.deepcopy(message.twist)
            if not all(
                math.isfinite(value) for value in forwarded_twist.covariance
            ):
                forwarded_twist.covariance = [0.0] * 36
                for index in (0, 7, 14, 21, 28, 35):
                    forwarded_twist.covariance[index] = 1.0e6

        self._latest_prior_twist = forwarded_twist
        self._latest_prior_twist_stamp_ns = (
            prior_stamp_ns if forwarded_twist is not None else None
        )
        self._prior_pose_buffer[prior_stamp_ns] = current_prior_pose
        self._prior_twist_buffer[prior_stamp_ns] = forwarded_twist
        CalibrationNode._prune_prior_buffers(self, prior_stamp_ns)
        CalibrationNode._drain_pending_scans(self)

    def _scan_points(self, scan: LaserScan) -> np.ndarray | None:
        if (
            scan.header.frame_id != self._lidar_frame
            or not math.isfinite(scan.angle_min)
            or not math.isfinite(scan.angle_increment)
            or scan.angle_increment == 0.0
            or not math.isfinite(scan.range_min)
            or not math.isfinite(scan.range_max)
            or scan.range_max <= scan.range_min
        ):
            return None

        ranges = np.asarray(scan.ranges, dtype=np.float64)
        indices = np.arange(0, len(ranges), self._scan_stride)
        selected = ranges[indices]
        valid = (
            np.isfinite(selected)
            & (selected >= float(scan.range_min))
            & (selected <= float(scan.range_max))
        )
        if int(np.count_nonzero(valid)) < self._fitter.minimum_matches:
            return None
        angles = float(scan.angle_min) + indices[valid] * float(scan.angle_increment)
        distances = selected[valid]
        return np.column_stack(
            (distances * np.cos(angles), distances * np.sin(angles))
        )

    def _clear_pending_recovery(self) -> None:
        self._pending_recovery_pose = None
        self._pending_recovery_prior = None
        self._pending_recovery_stamp_ns = None
        self._pending_recovery_count = 0

    def _reset_recovery_tracking(self) -> None:
        self._consecutive_fit_failures = 0
        CalibrationNode._clear_pending_recovery(self)

    def _reset_tracking(self) -> None:
        self._pose = self._initial_pose
        self._tracking_initialized = False
        self._latest_prior_pose = None
        self._latest_prior_stamp_ns = None
        self._latest_prior_twist = None
        self._latest_prior_twist_stamp_ns = None
        self._prior_pose_buffer.clear()
        self._prior_twist_buffer.clear()
        self._pending_scans.clear()
        self._tracking_pose_anchor = None
        self._tracking_prior_anchor = None
        self._last_fit_stamp_ns = None
        self._last_fit_mode = None
        self._last_stamp_ns = None
        CalibrationNode._reset_recovery_tracking(self)

    def _fresh_prior(self, stamp_ns: int) -> bool:
        return CalibrationNode._prior_pose_for_stamp(self, stamp_ns) is not None

    def _fresh_twist(self, stamp_ns: int) -> bool:
        return CalibrationNode._prior_twist_for_stamp(self, stamp_ns) is not None

    def _fit_initial_poses(
        self, stamp_ns: int
    ) -> tuple[tuple[float, float, float], ...]:
        if not self._tracking_initialized:
            return (self._initial_pose,)

        predicted = self._fallback_pose(stamp_ns)
        if predicted is not None and predicted != self._pose:
            return (predicted, self._pose)
        return (self._pose,)

    def _fallback_pose(
        self, stamp_ns: int
    ) -> tuple[float, float, float] | None:
        current_prior = CalibrationNode._prior_pose_for_stamp(self, stamp_ns)
        if (
            not self._tracking_initialized
            or self._tracking_pose_anchor is None
            or self._tracking_prior_anchor is None
            or current_prior is None
        ):
            return None
        return propagate_pose_with_relative_motion(
            self._tracking_pose_anchor,
            self._tracking_prior_anchor,
            current_prior,
        )

    def _fallback_budget_available(self, stamp_ns: int) -> bool:
        if self._last_fit_stamp_ns is None:
            return False
        elapsed_ns = stamp_ns - self._last_fit_stamp_ns
        return 0 <= elapsed_ns <= self._maximum_fallback_duration_ns

    def _confirm_recovery_candidate(
        self,
        candidate_pose: tuple[float, float, float],
        stamp_ns: int,
    ) -> bool:
        current_prior = CalibrationNode._prior_pose_for_stamp(self, stamp_ns)
        if current_prior is None:
            CalibrationNode._clear_pending_recovery(self)
            return False

        pending_stamp_ns = self._pending_recovery_stamp_ns
        if pending_stamp_ns is not None and stamp_ns <= pending_stamp_ns:
            if stamp_ns < pending_stamp_ns:
                CalibrationNode._clear_pending_recovery(self)
            return False

        consistent = False
        if (
            self._pending_recovery_pose is not None
            and self._pending_recovery_prior is not None
        ):
            expected_pose = propagate_pose_with_relative_motion(
                self._pending_recovery_pose,
                self._pending_recovery_prior,
                current_prior,
            )
            position_m, yaw_rad = _pose_correction(
                expected_pose, candidate_pose
            )
            consistent = (
                position_m <= self._recovery_consistency_position_m
                and yaw_rad <= self._recovery_consistency_yaw_rad
            )

        if consistent:
            self._pending_recovery_count += 1
        else:
            self._pending_recovery_count = 1
        self._pending_recovery_pose = candidate_pose
        self._pending_recovery_prior = current_prior
        self._pending_recovery_stamp_ns = stamp_ns
        return self._pending_recovery_count >= self._recovery_confirmation_frames

    def _accept_pose(
        self, pose: tuple[float, float, float], stamp_ns: int
    ) -> None:
        self._pose = pose
        self._tracking_initialized = True
        self._last_fit_stamp_ns = stamp_ns
        CalibrationNode._reset_recovery_tracking(self)
        current_prior = CalibrationNode._prior_pose_for_stamp(self, stamp_ns)
        if current_prior is not None:
            self._tracking_pose_anchor = pose
            self._tracking_prior_anchor = current_prior
        else:
            # Never reuse an anchor from an older fit after recovery.
            self._tracking_pose_anchor = None
            self._tracking_prior_anchor = None

    def _fit(self, points: np.ndarray, stamp_ns: int) -> FitResult | None:
        self._last_fit_mode = None
        initial_poses = self._fit_initial_poses(stamp_ns)
        result = self._fitter.fit_first(
            points, initial_poses, self._maximum_rms_error_m
        )
        if result is None and self._two_wall_fitter is not None:
            result = self._two_wall_fitter.fit_first(
                points, initial_poses, self._two_wall_maximum_rms_error_m
            )
        if result is not None:
            self._last_fit_mode = "regular"
            CalibrationNode._reset_recovery_tracking(self)
            return result

        predicted_pose = self._fallback_pose(stamp_ns)
        if (
            self._two_wall_fitter is not None
            and self._yaw_recovery_enabled
            and predicted_pose is not None
        ):
            result = fit_with_yaw_recovery(
                self._two_wall_fitter,
                points,
                predicted_pose,
                self._yaw_recovery_offsets_rad,
                self._two_wall_maximum_rms_error_m,
                self._two_wall_maximum_position_step_m,
                self._two_wall_maximum_yaw_step_rad,
                self._yaw_recovery_rms_tie_tolerance_m,
            )
            if result is not None:
                self._last_fit_mode = "yaw_recovery"
                CalibrationNode._reset_recovery_tracking(self)
                return result

        self._consecutive_fit_failures += 1
        if (
            not self._fallback_recovery_enabled
            or self._consecutive_fit_failures
            < self._fallback_recovery_trigger_frames
        ):
            CalibrationNode._clear_pending_recovery(self)
            return None
        if CalibrationNode._prior_pose_for_stamp(self, stamp_ns) is None:
            CalibrationNode._clear_pending_recovery(self)
            return None
        recovery_reference = (
            predicted_pose if predicted_pose is not None else self._pose
        )

        result = fit_with_bounded_pose_recovery(
            self._fitter,
            points,
            recovery_reference,
            xy_offsets_m=self._fallback_recovery_xy_offsets_m,
            yaw_offsets_rad=self._fallback_recovery_yaw_offsets_rad,
            maximum_rms_error_m=self._maximum_rms_error_m,
            maximum_position_correction_m=(
                self._fallback_recovery_maximum_position_correction_m
            ),
            maximum_yaw_correction_rad=(
                self._fallback_recovery_maximum_yaw_correction_rad
            ),
            rms_tie_tolerance_m=self._yaw_recovery_rms_tie_tolerance_m,
            ambiguity_position_m=self._recovery_ambiguity_position_m,
            ambiguity_yaw_rad=self._recovery_ambiguity_yaw_rad,
        )
        if result is None:
            CalibrationNode._clear_pending_recovery(self)
            return None
        confirmed = CalibrationNode._confirm_recovery_candidate(
            self, result.pose, stamp_ns
        )
        if not confirmed:
            return None
        self._last_fit_mode = "bounded_recovery"
        CalibrationNode._reset_recovery_tracking(self)
        return result

    def _on_scan(self, scan: LaserScan) -> None:
        stamp_ns = int(scan.header.stamp.sec) * 1_000_000_000 + int(
            scan.header.stamp.nanosec
        )
        if self._last_stamp_ns is not None and stamp_ns < self._last_stamp_ns:
            CalibrationNode._reset_tracking(self)
            self.get_logger().warning("scan time moved backwards; reset initial pose")
        elif self._last_stamp_ns is not None and stamp_ns == self._last_stamp_ns:
            self.get_logger().warning(
                "duplicate /scan timestamp ignored",
                throttle_duration_sec=2.0,
            )
            return
        self._last_stamp_ns = stamp_ns
        self._pending_scans[stamp_ns] = (scan, time.monotonic_ns())
        CalibrationNode._drain_pending_scans(self)

    def _process_scan(self, scan: LaserScan) -> None:
        stamp_ns = int(scan.header.stamp.sec) * 1_000_000_000 + int(
            scan.header.stamp.nanosec
        )

        points = self._scan_points(scan)
        if points is None:
            self._last_fit_mode = None
            self._consecutive_fit_failures += 1
            CalibrationNode._clear_pending_recovery(self)
            fallback_pose = self._fallback_pose(stamp_ns)
            if fallback_pose is None:
                self.get_logger().warning(
                    "invalid /scan and odometry fallback unavailable",
                    throttle_duration_sec=2.0,
                )
                return
            self._pose = fallback_pose
            if not CalibrationNode._fallback_budget_available(self, stamp_ns):
                self.get_logger().warning(
                    "invalid /scan; fallback budget exhausted",
                    throttle_duration_sec=2.0,
                )
                return
            self._publisher.publish(self._odometry(scan, None))
            self.get_logger().warning(
                "invalid /scan; publishing odometry fallback",
                throttle_duration_sec=2.0,
            )
            return

        result = self._fit(points, stamp_ns)
        if result is None:
            fallback_pose = self._fallback_pose(stamp_ns)
            if fallback_pose is not None:
                self._pose = fallback_pose
                if not CalibrationNode._fallback_budget_available(self, stamp_ns):
                    self.get_logger().warning(
                        "rectangle wall fit rejected; fallback budget exhausted",
                        throttle_duration_sec=2.0,
                    )
                    return
                self._publisher.publish(self._odometry(scan, None))
                self.get_logger().warning(
                    "rectangle wall fit rejected; publishing odometry fallback",
                    throttle_duration_sec=2.0,
                )
                return
            self.get_logger().warning(
                "rectangle wall fit rejected and odometry fallback unavailable",
                throttle_duration_sec=2.0,
            )
            return

        self._accept_pose(result.pose, stamp_ns)
        self._publisher.publish(self._odometry(scan, result.rms_error_m))

    def _odometry(
        self, scan: LaserScan, rms_error_m: float | None
    ) -> Odometry:
        x, y, yaw = ego_pose_from_lidar(
            self._pose, self._lidar_offset_x_m
        )
        message = Odometry()
        message.header.stamp = scan.header.stamp
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.child_frame_id = self._lidar_frame
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if rms_error_m is None:
            position_variance = self._fallback_position_variance_m2
            yaw_variance = self._fallback_yaw_variance_rad2
        else:
            position_variance = max(rms_error_m * rms_error_m, 1.0e-4)
            yaw_variance = position_variance
        message.pose.covariance[0] = position_variance
        message.pose.covariance[7] = position_variance
        message.pose.covariance[14] = 1.0e6
        message.pose.covariance[21] = 1.0e6
        message.pose.covariance[28] = 1.0e6
        message.pose.covariance[35] = yaw_variance
        scan_stamp_ns = int(scan.header.stamp.sec) * 1_000_000_000 + int(
            scan.header.stamp.nanosec
        )
        prior_twist = CalibrationNode._prior_twist_for_stamp(
            self, scan_stamp_ns
        )
        if prior_twist is not None:
            message.twist = copy.deepcopy(prior_twist)
        else:
            for index in (0, 7, 14, 21, 28, 35):
                message.twist.covariance[index] = 1.0e6
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

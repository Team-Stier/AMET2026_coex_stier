from __future__ import annotations

import math

import numpy as np

from calibration.pose_geometry import normalize_angle


class PoseCorrectionEkf:
    """Fuse lane pose measurements while propagating only odometry motion."""

    def __init__(
        self,
        *,
        process_position_variance_per_sec: float = 0.0025,
        process_yaw_variance_per_sec: float = 0.0016,
        minimum_measurement_position_variance: float = 0.0025,
        measurement_yaw_variance: float = 0.0076,
        maximum_output_position_rate_m_s: float = 0.08,
        maximum_output_yaw_rate_rad_s: float = 0.08,
    ) -> None:
        self.process_noise = np.diag(
            [
                process_position_variance_per_sec,
                process_position_variance_per_sec,
                process_yaw_variance_per_sec,
            ]
        )
        self.minimum_measurement_position_variance = (
            minimum_measurement_position_variance
        )
        self.measurement_yaw_variance = measurement_yaw_variance
        self.maximum_output_position_rate_m_s = maximum_output_position_rate_m_s
        self.maximum_output_yaw_rate_rad_s = maximum_output_yaw_rate_rad_s
        self.reset()

    def reset(self) -> None:
        self.state: np.ndarray | None = None
        self.output_state: np.ndarray | None = None
        self.covariance = np.diag([0.04, 0.04, 0.03]).astype(np.float64)
        self.previous_raw_pose: np.ndarray | None = None
        self.last_innovation = np.full(3, np.nan, dtype=np.float64)
        self.last_gain_diagonal = np.full(3, np.nan, dtype=np.float64)

    @staticmethod
    def _planar_covariance(values) -> np.ndarray | None:
        array = np.asarray(values, dtype=np.float64)
        if array.size != 36 or not np.all(np.isfinite(array)):
            return None
        matrix = array.reshape(6, 6)
        planar = matrix[np.ix_([0, 1, 5], [0, 1, 5])]
        planar = 0.5 * (planar + planar.T)
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(planar)
        except np.linalg.LinAlgError:
            return None
        eigenvalues = np.clip(eigenvalues, 1.0e-9, 1.0e3)
        return eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T

    def output_covariance(self) -> np.ndarray:
        """Return posterior covariance plus unapplied rate-limit displacement."""
        result = self.covariance.copy()
        if self.state is None or self.output_state is None:
            return result
        lag = self.state - self.output_state
        lag[2] = normalize_angle(float(lag[2]))
        return result + np.diag(lag * lag)

    @property
    def output_pose(self) -> tuple[float, float, float] | None:
        if self.output_state is None:
            return None
        return tuple(float(value) for value in self.output_state)

    @staticmethod
    def _propagate_pose(
        pose: np.ndarray,
        previous_raw: np.ndarray,
        current_raw: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        raw_dx = current_raw[0] - previous_raw[0]
        raw_dy = current_raw[1] - previous_raw[1]
        previous_cosine = math.cos(previous_raw[2])
        previous_sine = math.sin(previous_raw[2])
        local_dx = previous_cosine * raw_dx + previous_sine * raw_dy
        local_dy = -previous_sine * raw_dx + previous_cosine * raw_dy
        delta_yaw = normalize_angle(current_raw[2] - previous_raw[2])

        yaw = pose[2]
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        propagated = np.array(
            [
                pose[0] + cosine * local_dx - sine * local_dy,
                pose[1] + sine * local_dx + cosine * local_dy,
                normalize_angle(yaw + delta_yaw),
            ],
            dtype=np.float64,
        )
        jacobian = np.eye(3, dtype=np.float64)
        jacobian[0, 2] = -sine * local_dx - cosine * local_dy
        jacobian[1, 2] = cosine * local_dx - sine * local_dy
        return propagated, jacobian

    def predict(
        self,
        raw_pose: tuple[float, float, float],
        dt_sec: float,
        *,
        pose_covariance=None,
        twist_covariance=None,
    ) -> None:
        raw = np.asarray(raw_pose, dtype=np.float64)
        if self.state is None or self.output_state is None or self.previous_raw_pose is None:
            self.state = raw.copy()
            self.output_state = raw.copy()
            self.previous_raw_pose = raw
            initial_covariance = self._planar_covariance(pose_covariance)
            if initial_covariance is not None:
                self.covariance = initial_covariance
            return

        self.state, jacobian = self._propagate_pose(
            self.state, self.previous_raw_pose, raw
        )
        self.output_state, _ = self._propagate_pose(
            self.output_state, self.previous_raw_pose, raw
        )
        dt = max(0.0, float(dt_sec))
        process_covariance = self.process_noise * dt
        velocity_covariance = self._planar_covariance(twist_covariance)
        if velocity_covariance is not None and dt > 0.0:
            yaw = float(self.state[2])
            cosine, sine = math.cos(yaw), math.sin(yaw)
            velocity_to_pose = np.array(
                [
                    [cosine, -sine, 0.0],
                    [sine, cosine, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            process_covariance += (
                velocity_to_pose
                @ velocity_covariance
                @ velocity_to_pose.T
                * dt
                * dt
            )
        self.covariance = (
            jacobian @ self.covariance @ jacobian.T + process_covariance
        )
        self.previous_raw_pose = raw

    def correct(
        self,
        measured_pose: tuple[float, float, float],
        *,
        rms_error_m: float,
        match_count: int,
    ) -> None:
        if self.state is None:
            return
        measurement = np.asarray(measured_pose, dtype=np.float64)
        match_scale = max(0.25, 8.0 / max(1, int(match_count)))
        position_variance = max(
            self.minimum_measurement_position_variance,
            float(rms_error_m) ** 2,
        ) * match_scale
        measurement_noise = np.diag(
            [
                position_variance,
                position_variance,
                self.measurement_yaw_variance * match_scale,
            ]
        )
        innovation = measurement - self.state
        innovation[2] = normalize_angle(float(innovation[2]))
        innovation_covariance = self.covariance + measurement_noise
        try:
            gain = np.linalg.solve(
                innovation_covariance.T, self.covariance.T
            ).T
        except np.linalg.LinAlgError:
            return
        self.last_innovation = innovation.copy()
        self.last_gain_diagonal = np.diag(gain).copy()
        self.state = self.state + gain @ innovation
        self.state[2] = normalize_angle(float(self.state[2]))
        identity = np.eye(3, dtype=np.float64)
        residual_gain = identity - gain
        self.covariance = (
            residual_gain @ self.covariance @ residual_gain.T
            + gain @ measurement_noise @ gain.T
        )

    def advance_output(self, dt_sec: float) -> None:
        if self.state is None or self.output_state is None:
            return
        dt = max(0.0, min(0.25, float(dt_sec)))
        delta_xy = self.state[:2] - self.output_state[:2]
        distance = float(np.linalg.norm(delta_xy))
        maximum_distance = self.maximum_output_position_rate_m_s * dt
        if distance > maximum_distance > 0.0:
            delta_xy *= maximum_distance / distance
        self.output_state[:2] += delta_xy

        delta_yaw = normalize_angle(float(self.state[2] - self.output_state[2]))
        maximum_yaw = self.maximum_output_yaw_rate_rad_s * dt
        delta_yaw = float(np.clip(delta_yaw, -maximum_yaw, maximum_yaw))
        self.output_state[2] = normalize_angle(float(self.output_state[2] + delta_yaw))

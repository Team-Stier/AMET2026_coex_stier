"""Small, ROS-independent data models used by the controller algorithms."""

from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class PathPoint:
    """A two-dimensional reference-path point in metres."""

    x: float
    y: float


@dataclass
class VehicleState:
    """Planar vehicle state in SI units."""

    x: float
    y: float
    yaw: float
    speed: float


@dataclass(frozen=True)
class PurePursuitConfig:
    """Pure Pursuit parameters in SI units."""

    wheelbase_m: float
    lookahead_distance_m: float
    max_steering_rad: float
    closed_loop: bool = False

    def __post_init__(self) -> None:
        if self.wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive")
        if self.lookahead_distance_m <= 0.0:
            raise ValueError("lookahead_distance_m must be positive")
        if self.max_steering_rad <= 0.0:
            raise ValueError("max_steering_rad must be positive")


@dataclass(frozen=True)
class PurePursuitResult:
    """Pure Pursuit command plus compact debugging information."""

    steering_rad: float
    target_point: PathPoint
    target_index: int
    nearest_point: PathPoint
    nearest_index: int
    alpha_rad: float
    target_distance_m: float


@dataclass(frozen=True)
class PIDConfig:
    """Longitudinal PID gains and correction limits in SI units."""

    kp: float
    ki: float
    kd: float
    output_min: float
    output_max: float
    integral_min: float
    integral_max: float

    def __post_init__(self) -> None:
        if self.output_min > self.output_max:
            raise ValueError("output_min must not exceed output_max")
        if self.integral_min > self.integral_max:
            raise ValueError("integral_min must not exceed integral_max")


@dataclass(frozen=True)
class PIDResult:
    """PID speed correction and terms used to produce it."""

    output: float
    error: float
    p_term: float
    i_term: float
    d_term: float
    integral: float
    derivative: float
    output_saturated: bool


@dataclass(frozen=True)
class AdaptiveControlConfig:
    """Curvature-based lookahead and speed-cap parameters in SI units."""

    enabled: bool = False
    preview_distance_m: float = 1.0
    min_lookahead_m: float = 0.25
    max_lookahead_m: float = 0.45
    curvature_reference_inv_m: float = 2.0
    max_lateral_acceleration_m_s2: float = 0.8
    min_speed_limit_m_s: float = 0.30
    max_speed_limit_m_s: float = 0.80

    def __post_init__(self) -> None:
        if self.preview_distance_m <= 0.0:
            raise ValueError("preview_distance_m must be positive")
        if self.min_lookahead_m <= 0.0:
            raise ValueError("min_lookahead_m must be positive")
        if self.min_lookahead_m > self.max_lookahead_m:
            raise ValueError("min_lookahead_m must not exceed max_lookahead_m")
        if self.curvature_reference_inv_m <= 0.0:
            raise ValueError("curvature_reference_inv_m must be positive")
        if self.max_lateral_acceleration_m_s2 <= 0.0:
            raise ValueError(
                "max_lateral_acceleration_m_s2 must be positive"
            )
        if self.min_speed_limit_m_s < 0.0:
            raise ValueError("min_speed_limit_m_s must not be negative")
        if self.min_speed_limit_m_s > self.max_speed_limit_m_s:
            raise ValueError(
                "min_speed_limit_m_s must not exceed max_speed_limit_m_s"
            )


@dataclass(frozen=True)
class AdaptiveControlResult:
    """Curvature metric and commands selected by the adaptive policy."""

    curvature_inv_m: float
    lookahead_distance_m: float
    speed_limit_m_s: float


@dataclass(frozen=True)
class ControllerConfig:
    """Shared controller behavior independent of any ROS transport."""

    longitudinal_pid_enabled: bool
    max_speed_m_s: float
    stop_speed_threshold_m_s: float = 1.0e-6
    adaptive_control: AdaptiveControlConfig = field(
        default_factory=AdaptiveControlConfig
    )

    def __post_init__(self) -> None:
        if self.max_speed_m_s <= 0.0:
            raise ValueError("max_speed_m_s must be positive")
        if self.stop_speed_threshold_m_s < 0.0:
            raise ValueError("stop_speed_threshold_m_s must not be negative")


@dataclass(frozen=True)
class ControllerResult:
    """Combined commands and algorithm debugging information."""

    steering_rad: float
    speed_command_m_s: float
    pure_pursuit: PurePursuitResult
    pid: Optional[PIDResult]
    adaptive: Optional[AdaptiveControlResult] = None


PathLike = Sequence[Tuple[float, float]]

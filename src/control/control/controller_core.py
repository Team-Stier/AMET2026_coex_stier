"""ROS-independent combined lateral and longitudinal controller."""

from typing import Sequence

from .adaptive_policy import AdaptiveControlPolicy
from .models import ControllerConfig, ControllerResult, VehicleState
from .pid import PIDController
from .pure_pursuit import PointInput, PurePursuit


class ControllerCore:
    """Shared controller facade used by simulation and future ROS2 adapters."""

    def __init__(
        self,
        pure_pursuit: PurePursuit,
        pid: PIDController,
        config: ControllerConfig,
    ) -> None:
        self.pure_pursuit = pure_pursuit
        self.pid = pid
        self.config = config
        self.adaptive_policy = AdaptiveControlPolicy(config.adaptive_control)

    def update(
        self,
        vehicle_state: VehicleState,
        path: Sequence[PointInput],
        target_speed: float,
        dt: float,
    ) -> ControllerResult:
        adaptive_result = None
        lookahead_override = None
        speed_limit = self.config.max_speed_m_s
        if self.config.adaptive_control.enabled:
            adaptive_result = self.adaptive_policy.compute(
                vehicle_state,
                path,
                closed_loop=self.pure_pursuit.config.closed_loop,
            )
            lookahead_override = adaptive_result.lookahead_distance_m
            speed_limit = adaptive_result.speed_limit_m_s

        lateral = self.pure_pursuit.compute(
            vehicle_state,
            path,
            lookahead_distance_override_m=lookahead_override,
        )
        bounded_target = _clamp(
            min(target_speed, speed_limit), 0.0, self.config.max_speed_m_s
        )

        if (
            self.config.longitudinal_pid_enabled
            and bounded_target <= self.config.stop_speed_threshold_m_s
        ):
            # A planner stop command is authoritative. Do not let signed odometry
            # noise create a correction or leave state that affects restart.
            self.pid.reset()
            pid_result = None
            speed_command = 0.0
        elif self.config.longitudinal_pid_enabled:
            pid_result = self.pid.compute(
                bounded_target, vehicle_state.speed, dt
            )
            speed_command = _clamp(
                bounded_target + pid_result.output,
                0.0,
                self.config.max_speed_m_s,
            )
        else:
            # Disabled mode is direct target-speed pass-through. Clearing state
            # also ensures a later enable starts without stale integral history.
            self.pid.reset()
            pid_result = None
            speed_command = bounded_target

        return ControllerResult(
            steering_rad=lateral.steering_rad,
            speed_command_m_s=speed_command,
            pure_pursuit=lateral,
            pid=pid_result,
            adaptive=adaptive_result,
        )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))

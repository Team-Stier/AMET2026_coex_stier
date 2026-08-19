"""ROS-independent combined lateral and longitudinal controller."""

from typing import Sequence

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

    def update(
        self,
        vehicle_state: VehicleState,
        path: Sequence[PointInput],
        target_speed: float,
        dt: float,
    ) -> ControllerResult:
        lateral = self.pure_pursuit.compute(vehicle_state, path)
        bounded_target = _clamp(target_speed, 0.0, self.config.max_speed_m_s)

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
        )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))

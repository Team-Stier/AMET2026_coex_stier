"""ROS-independent PID producing an upper-level speed correction."""

from typing import Optional

from .models import PIDConfig, PIDResult


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class PIDController:
    """Stateful PID whose output is a speed correction in m/s."""

    def __init__(self, config: PIDConfig) -> None:
        self.config = config
        self._integral = 0.0
        self._previous_error = None  # type: Optional[float]

    @property
    def integral(self) -> float:
        return self._integral

    @property
    def previous_error(self) -> Optional[float]:
        return self._previous_error

    def compute(
        self, target_speed: float, current_speed: float, dt: float
    ) -> PIDResult:
        """Return a bounded correction for ``target_speed - current_speed``.

        A non-positive dt leaves the time-dependent state unchanged. The first
        valid sample also uses zero derivative to avoid a startup spike.
        """

        error = target_speed - current_speed
        derivative = 0.0
        candidate_integral = self._integral

        if dt > 0.0:
            candidate_integral = _clamp(
                self._integral + error * dt,
                self.config.integral_min,
                self.config.integral_max,
            )
            if self._previous_error is not None:
                derivative = (error - self._previous_error) / dt

        p_term = self.config.kp * error
        d_term = self.config.kd * derivative
        candidate_i_term = self.config.ki * candidate_integral
        candidate_output = p_term + candidate_i_term + d_term

        # Conditional integration: reject a new integral contribution only when
        # the output is saturated and the current error would push it farther
        # into that same saturation limit.
        drives_high = candidate_output > self.config.output_max and error > 0.0
        drives_low = candidate_output < self.config.output_min and error < 0.0
        if dt > 0.0 and (drives_high or drives_low):
            candidate_integral = self._integral

        if dt > 0.0:
            self._integral = candidate_integral
            self._previous_error = error

        i_term = self.config.ki * self._integral
        raw_output = p_term + i_term + d_term
        output = _clamp(raw_output, self.config.output_min, self.config.output_max)
        return PIDResult(
            output=output,
            error=error,
            p_term=p_term,
            i_term=i_term,
            d_term=d_term,
            integral=self._integral,
            derivative=derivative,
            output_saturated=output != raw_output,
        )

    def reset(self) -> None:
        """Clear accumulated integral and derivative history."""

        self._integral = 0.0
        self._previous_error = None

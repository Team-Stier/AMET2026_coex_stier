# Control package

This package keeps path-following and longitudinal controller algorithms independent
of ROS, simulator APIs, and other team packages. ROS and simulator adapters must
convert their inputs to the plain Python models before calling `ControllerCore`.

## Control feature defaults

REAL baseline:

- `closed_loop = false`
- PID = false
- Adaptive Control = false until real validation

Current SIM mock-route experiment:

- `closed_loop = true`
- PID = false
- Adaptive Control = true for A/B test
- planner target speed = 0.80 m/s
- preview distance = 1.0 m
- min lookahead = 0.25 m
- max lookahead = 0.40 m
- curvature reference = 2.0 1/m
- maximum lateral acceleration = 0.8 m/s²
- curvature speed-limit range = 0.30–0.80 m/s

Adaptive Control computes a stateless preview-path curvature on every update. It
uses that metric to reduce lookahead and to provide a curvature speed cap. The
speed policy follows the lateral-acceleration relation and is clamped to the
configured speed-limit range:

```text
v_curve = sqrt(max_lateral_acceleration / curvature)
```

Near-zero curvature uses the maximum speed limit. The policy is only an upper
limit and never raises the planner target:

```text
effective target speed = min(planner target speed, curvature speed limit)
```

For the first speed-policy SIM experiment, the planner target is 0.80 m/s and
curvature may reduce the command as low as the configured 0.30 m/s limit. The
0.30 m/s value is an initial SIM experiment parameter, not a fixed minimum speed
embedded in the controller core. It must be tuned from measured tracking and
lateral behavior.

## SIM and REAL separation

The SIM mock route is an explicitly closed path and may enable Adaptive Control
for experimentation. REAL remains open-path, PID-off, and Adaptive-off by default
until planner/localization interfaces and real-vehicle validation are complete.
Simulator-only route/pose APIs belong in a future adapter and must not be imported
by `models.py`, `geometry.py`, `path_metrics.py`, `adaptive_policy.py`,
`pure_pursuit.py`, `pid.py`, or `controller_core.py`.

Keep all environment-specific enable/disable parameters and test results documented
here as the SIM and REAL configurations evolve.

## SIM Autotune Result (2026-08-20)

- 72 experiments
- 63/63 successful lap experiments
- `OPTIMUM_CONVERGED`
- min lookahead 0.25 m
- max lookahead 0.40 m
- curvature reference 2.0 1/m
- preview distance 1.0 m
- max lateral acceleration 0.8 m/s²
- longitudinal PID OFF
- final validation 3/3 laps
- mean path error ≈ 0.0186 m
- max path error ≈ 0.0456 m
- max steering ≈ 19.54 deg
- steering saturation 0

These results were obtained on the PhysiCar SIM kinematically feasible mock path.
They must be revalidated on the actual Hybrid A* planner path and the real vehicle.

# Control module

This package provides a ROS-independent `ControllerCore` for vehicle path
following. ROS nodes and other adapters convert their inputs to the plain Python
models before calling the controller.

## Features

- Pure Pursuit lateral control
- curvature-adaptive lookahead
- curvature-based speed limiting
- optional longitudinal PID
- ROS-independent `ControllerCore`

## Current defaults

- configuration: `config/control.yaml`
- target speed: 0.55 m/s
- hardware speed cap: 3.0 m/s
- speed lookahead: `clamp(0.55 × |speed|, 0.45, 1.50)` m
- lookahead at 0.55 m/s: 0.45 m
- lookahead at 1.5 m/s: 0.825 m
- Pure Pursuit reference point: 0.027 m forward from `lidar_link`
- steering hard limit: 0.3491 rad (approximately 20°)
- curvature-adaptive lookahead bounds: 0.25–1.50 m
- preview distance: 1.0 m
- curvature reference: 2.0 1/m
- maximum lateral acceleration: 0.8 m/s²
- adaptive speed range: 0.30–0.80 m/s
- longitudinal PID: OFF by default

`run.sh` loads the installed `control.yaml` automatically. Change controller
parameters in the source YAML and rebuild before running. `target_speed_m_s`
sets the normal command, while `max_speed_m_s` is the final safety clamp.

`pure_pursuit.reference_point_x_from_lidar_m` moves the point used for the
Pure Pursuit nearest-point, target-distance, and target-heading calculations.
The value is a signed longitudinal position from the LiDAR origin: positive is
forward, `0.0` is the LiDAR origin, and negative is behind the LiDAR. The
default `+0.027 m` preserves the current `/pose/calibration` reference point.
`calibrated_pose.origin_x_from_lidar_m` is the compatibility contract for the
incoming pose origin, not a tuning value. It must be updated with the
Calibration LiDAR extrinsic; set it to `0.0` if `/pose/calibration.pose.position`
is changed to the actual `lidar_link` origin.

## Controller interface

Call `ControllerCore.update(vehicle_state, path, target_speed, dt)`.

Inputs:

- `VehicleState`
- planner path
- target speed
- time step `dt`

Outputs (`ControllerResult`):

- steering command (`steering_rad`)
- speed command (`speed_command_m_s`)

The adaptive speed limit is an upper bound and does not raise the planner target
speed. Enable adaptive control or longitudinal PID through the existing
`ControllerConfig` policy; the current enable/disable behavior is unchanged.

Revalidate the controller on the final planner path and the real vehicle before
deployment.

# PhysiCar lane-calibration recording

## Recording identity

- Recorded (UTC): 2026-08-22 17:45:23–17:50:03
- Workspace: `/home/physicar/physicar_ws`
- Git top-level: `/home/physicar/physicar_ws`
- Branch: `feature/Paik-calibrabtion`
- Commit: `b585f238163fdc7782f20f94d9f6da006cb97a20`
- ROS distribution: Jazzy
- Overlay: `/home/physicar/physicar_ws/install` over `/opt/ros/jazzy`
- Calibration configuration: `/home/physicar/physicar_ws/src/calibration/config/lane_detection.yaml`
- Bag: `/home/physicar/physicar_ws/records/calibration/true_pose_lane_calibration_20260822_174517`
- Storage: MCAP, 272,333,720 bytes (259.7 MiB reported by `ros2 bag info`)
- Duration: 280.664536019 s
- Total messages: 209,998

The calibration source and configuration were not changed. The existing unrelated
working-tree changes under `src/traffic_light`, `AMET2026_coex_stier`, `app.physicar`,
and `graphify-out` were left untouched. The bag and this note are untracked and were
not added to Git or committed.

## Simulator and route

- Simulator world: `custom_e09090b056ef1f90f845419690065271`
- Driver source: repository revision `3a4074b`,
  `src/control/tools/sim_waypoint_mapping_drive.py`
- Waypoint source: repository revision `3a4074b`,
  `src/control/config/sim_mapping_waypoints.json`
- Logical waypoint count: 668 (669 JSON points including the duplicated closing point)
- Closed-route length: 32.259 m
- Route metadata world matched the running simulator world.
- Completed laps: 3.00
- Main-drive elapsed time: 194.3 s
- Maximum nearest-waypoint error: 0.051 m

The simulator was fully respawned before starting the calibration and recording
processes. The bag contains 6.02 s stationary before the first non-zero command.
The initial straight phase used a 0.02 m/s command for a nominal 60 s (59.598 s of
positive commands after preflight). Ground truth moved from `(1.39998, 3.40210)` to
`(1.40756, 2.24672)` m while yaw changed only 0.574 degrees. The vehicle remained on
the same straight through the subsequent safety stop, so the departure-to-first-turn
interval exceeds 60 s. The three counted laps then used a 0.55 m/s target with
curvature limiting (typically 0.50 m/s in turns). The route includes straight,
left/right turns, and tight curves; camera lane visibility varies naturally around it.

## Ground truth selection

Selected pose topic: `/sim/ground_truth/tf` (`tf2_msgs/msg/TFMessage`), bridged
directly from Gazebo `/model/physicar/pose` (`gz.msgs.Pose_V`). It was selected over
`/odom`, `/odom/laser`, RViz paths, and waypoint data because it is the simulator's
direct model-pose stream and carries source simulation timestamps, absolute XYZ, and
the full orientation quaternion at about 48 Hz.

Representative first sample:

```yaml
header.stamp: 33.120000000
header.frame_id: odom
child_frame_id: base_footprint
translation: {x: 1.3999762274, y: 3.4021045869, z: -0.0000002284}
rotation: {x: 1.61e-9, y: 1.61e-9, z: -0.7069819945, w: 0.7072315459}
```

Important frame note: the Gazebo message labels this transform `odom ->
base_footprint`, but its numeric pose is the simulator-world pose: it equals
`GET /sim/api/pose` and begins at the route's world spawn `(1.399976, 3.402105)`,
while `/model/physicar/odometry` begins near local `(0, 0)`. Therefore analysis must
treat `/sim/ground_truth/tf` translation/orientation as `sim_world -> vehicle` and
must not inject it into `/tf` under the misleading source labels.

`/sim/ground_truth/odometry` (`nav_msgs/msg/Odometry`) is also recorded. It is the
Gazebo odometry publisher's local pose/twist stream (`physicar/odom` ->
`physicar/base_footprint`) and supplies simulator true velocity. `/mapping/sim_pose`
is the control driver's independent 20 Hz API-polled world-pose copy and is retained
as a cross-check, not as the primary truth stream.

The physical world/map alignment used in this run is the fixed SE(2) transform
published by the existing `calibration_sim_rviz_bridge` from simultaneous simulator
world pose and raw odom:

```yaml
map_to_odom:
  translation: {x: 1.3977102083, y: 3.3959959142, z: 0.0}
  rotation: {x: 0.0, y: 0.0, z: -0.7068320827, w: 0.7073813730}
```

No screenshot homography or arbitrary scale is used for pose conversion.

## ROS graph and TF checks

- `/clock`: one publisher (`ros_gz_bridge`)
- `/odom/calibride`: one publisher (`calibration_node`)
- `map -> odom`: one static publisher (`calibration_sim_rviz_bridge`)
- No competing publisher was found for any recorded child frame.

Complete vehicle/camera chain present in the bag:

```text
map
└── odom                         static, 1 sample
    └── base_footprint           dynamic, 8,070 samples
        └── base_link            static, 1 sample
            └── camera_pan_link  dynamic, 5,380 samples
                └── camera_tilt_link  dynamic, 5,380 samples
                    └── camera_link   static, 1 sample
                        └── camera_optical_frame  static, 1 sample
```

Additional static frames `base_link -> imu_link` and `base_link -> lidar_link` are
also present. Every individual TF edge has monotonically nondecreasing timestamps.
The aggregate `/tf` message sequence reports 499 apparent reversals and `/tf_static`
one apparent reversal only because messages from different publishers/edges are
interleaved; per-edge checks report zero reversals.

## Measured rates

The required preflight measurement ran for 12 s before recording:

| Stream | Preflight rate |
|---|---:|
| camera compressed | 14.698 Hz |
| raw `/odom` | 29.193 Hz |
| simulator true pose | 48.579 Hz |
| IMU | 48.698 Hz |
| TF messages | 48.661 Hz |
| `/odom/calibride` | 29.166 Hz |

Whole-bag effective rates from first to last record were 14.521 Hz camera, 28.751 Hz
raw odom, 47.918 Hz true pose, 47.918 Hz IMU, 47.921 Hz TF, and 28.762 Hz calibrated
odom. Header timestamps increased normally for camera, odom, calibrated odom, IMU,
ground truth, driver pose, and clock.

## Recorded topics and counts

| Topic | Type | Count |
|---|---|---:|
| `/clock` | `rosgraph_msgs/msg/Clock` | 53,796 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 13,450 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 2 |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 4,076 |
| `/odom` | `nav_msgs/msg/Odometry` | 8,069 |
| `/odom/laser` | `nav_msgs/msg/Odometry` | 2,690 |
| `/odom/calibride` | `nav_msgs/msg/Odometry` | 8,070 |
| `/imu` | `sensor_msgs/msg/Imu` | 13,449 |
| `/speed` | `std_msgs/msg/Float64` | 5,091 |
| `/steering` | `std_msgs/msg/Float64` | 5,092 |
| `/calibration/detected_centerline` | `nav_msgs/msg/Path` | 1,272 |
| `/sim/ground_truth/tf` | `tf2_msgs/msg/TFMessage` | 13,449 |
| `/sim/ground_truth/odometry` | `nav_msgs/msg/Odometry` | 13,449 |
| `/mapping/sim_pose` | `geometry_msgs/msg/PoseStamped` | 5,080 |
| `/camera/pan` | `std_msgs/msg/Float64` | 5,091 |
| `/camera/tilt` | `std_msgs/msg/Float64` | 0 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 53,795 |
| `/calibration/debug/bev/compressed` | `sensor_msgs/msg/CompressedImage` | 1,359 |
| `/calibration/debug/lane_mask/compressed` | `sensor_msgs/msg/CompressedImage` | 1,359 |
| `/calibration/debug/lane_overlay/compressed` | `sensor_msgs/msg/CompressedImage` | 1,359 |

`/camera/tilt` existed but emitted no command messages. Actual pan/tilt state is
available throughout `/joint_states` as `camera_pan_joint` and `camera_tilt_joint`.
There was no running global waypoint/path topic, so none was invented or recorded.

## Timestamp alignment

All values use message header timestamps and nearest-neighbor matching at every
camera timestamp:

| Difference | Mean | P95 | Max |
|---|---:|---:|---:|
| camera to `/odom` | 8.940 ms | 20.000 ms | 50.000 ms |
| camera to simulator true pose | 4.909 ms | 10.000 ms | 50.000 ms |
| selected odom to selected truth | 8.047 ms | 20.000 ms | 20.000 ms |
| camera/odom/truth triplet span | 10.948 ms | 20.000 ms | 50.000 ms |

Header ranges were camera `33.070..302.020`, odom `33.120..302.060`, true pose
`33.120..302.080`, and clock `33.105..302.080` seconds in simulator time. True pose
is present from the beginning through the end of the recording.

## Data integrity

- No NaN or Inf was found in raw odom, calibrated odom, laser odom, primary true
  pose, simulator odometry, or driver API pose.
- Quaternion norm was 1.000000000 (min/max at printed precision) for all 55,587
  checked pose samples.
- `/tf_static`, `map -> odom`, and the complete camera TF chain are present.
- `ros2 bag info` completed successfully.
- A one-second playback smoke test of `/sim/ground_truth/tf` completed with exit 0.
- The recorder was stopped with SIGINT and wrote metadata normally.

## Exact recording command

```bash
ros2 bag record \
  -o /home/physicar/physicar_ws/records/calibration/true_pose_lane_calibration_20260822_174517 \
  /clock /tf /tf_static /camera/image_raw/compressed \
  /odom /odom/laser /odom/calibride /imu /speed /steering \
  /calibration/detected_centerline \
  /sim/ground_truth/tf /sim/ground_truth/odometry /mapping/sim_pose \
  /camera/pan /camera/tilt /joint_states \
  /calibration/debug/bev/compressed \
  /calibration/debug/lane_mask/compressed \
  /calibration/debug/lane_overlay/compressed
```

## Replay

Evaluate the already-recorded calibrated odom without running calibration or another
`map -> odom` publisher:

```bash
ros2 bag play /home/physicar/physicar_ws/records/calibration/true_pose_lane_calibration_20260822_174517
```

The bag already contains `/clock`; do **not** add `ros2 bag play --clock`.

To regenerate `/odom/calibride` with the current code, start `calibration_node` only
(do not start `sim_rviz_bridge`) and exclude every recorded calibration output:

```bash
ros2 bag play /home/physicar/physicar_ws/records/calibration/true_pose_lane_calibration_20260822_174517 \
  --exclude-topics /odom/calibride \
  /calibration/detected_centerline \
  /calibration/debug/bev/compressed \
  /calibration/debug/lane_mask/compressed \
  /calibration/debug/lane_overlay/compressed
```

The recorded `/tf_static` supplies the single `map -> odom` transform during that
replay. Do not run any other node that broadcasts the same transform.

## Known warnings

- The Gazebo primary truth stream's source frame labels are misleading as described
  above; use the documented simulator-world interpretation.
- `/camera/tilt` has zero messages, but `/joint_states` contains the actual tilt state.
- A superseded first recording exists at
  `/home/physicar/physicar_ws/records/calibration/true_pose_lane_calibration_20260822_173752`;
  its initial 60-second low-speed phase entered the first bend and it should not be
  used for the requested initial-straight analysis.

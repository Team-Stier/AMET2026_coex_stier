# Odometry and LiDAR comparison bag

## Streams kept separate

| Topic | Type | Meaning |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | `ekf_filter_node` fused odometry |
| `/odom/laser` | `nav_msgs/msg/Odometry` | `laser_odom` LiDAR odometry |
| `/odom/calibride` | `nav_msgs/msg/Odometry` | fence-corrected output under test |
| `/sim/ground_truth/tf` | `tf2_msgs/msg/TFMessage` | simulator absolute world pose |
| `/sim/ground_truth/odometry` | `nav_msgs/msg/Odometry` | simulator local truth pose and twist |
| `/scan` | `sensor_msgs/msg/LaserScan` | raw LiDAR ranges in `lidar_link` |
| `/scan_filtered` | `sensor_msgs/msg/LaserScan` | range-filtered LiDAR scan |

`/odom` is not raw wheel odometry and is not simulator ground truth. It is the
localization EKF output. `/odom/laser` is the independent LiDAR odometry estimate.
The recorded covariance of `/odom/laser` must be inspected; zero-filled covariance
does not prove zero uncertainty.

## Recording

Start the simulator, sensors, truth bridge, calibration node, and route driver.
Verify that there is exactly one publisher for each odometry output, then run:

```bash
cd /home/physicar/physicar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
src/calibration/tools/record_calibration_diagnostic_bag.sh three_lap_lidar_compare
```

The script refuses to record unless `/scan`, `/scan_filtered`, `/odom`,
`/odom/laser`, and `/sim/ground_truth/tf` are present with the expected message
types. It records raw/filtered LiDAR, all odometry variants, truth, IMU, TF,
commands, joint states, and simulation time into one MCAP bag.

Before driving, check rates and publishers in another terminal:

```bash
ros2 topic info /odom --verbose
ros2 topic info /odom/laser --verbose
ros2 topic info /odom/calibride --verbose
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /odom/laser
ros2 topic echo /scan --once --field header
```

For a useful comparison, include the initial stationary period, the same initial
straight used in prior tests, and at least three complete laps with turns. Do not
replay an older bag while recording, because that creates competing publishers and
mixes old and live sensor messages.

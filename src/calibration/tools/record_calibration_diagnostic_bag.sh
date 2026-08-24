#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
RECORDS_ROOT="${RECORDS_ROOT:-$WORKSPACE_ROOT/records/calibration}"
RUN_LABEL="${1:-odom_lidar_comparison}"

if [[ ! "$RUN_LABEL" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "error: run label may contain only letters, numbers, dot, underscore, and dash" >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    if [[ ! -f "$ROS_SETUP" ]]; then
        echo "error: ros2 not found and ROS setup is missing: $ROS_SETUP" >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    set +u
    source "$ROS_SETUP"
    set -u
fi

declare -A required_types=(
    ["/scan"]="sensor_msgs/msg/LaserScan"
    ["/odom"]="nav_msgs/msg/Odometry"
    ["/odom/laser"]="nav_msgs/msg/Odometry"
    ["/camera/image_raw/compressed"]="sensor_msgs/msg/CompressedImage"
    ["/sim/ground_truth/tf"]="tf2_msgs/msg/TFMessage"
)

topic_table="$(ros2 topic list --show-types)"
missing=0
echo "Required calibration diagnostic topics"
for topic in "${!required_types[@]}"; do
    expected_type="${required_types[$topic]}"
    if grep -Fqx "$topic [$expected_type]" <<<"$topic_table"; then
        echo "  OK      $topic [$expected_type]"
    else
        echo "  MISSING $topic [$expected_type]"
        missing=1
    fi
done

if ((missing != 0)); then
    echo "error: required topics are missing; start the simulator sensor stack first" >&2
    exit 2
fi

declare -a topics=(
    # Odometry sources kept separately for direct timestamped comparison.
    "/odom"
    "/odom/laser"
    "/odom/calibride"
    "/sim/ground_truth/odometry"
    "/sim/ground_truth/tf"
    "/mapping/sim_pose"

    # Raw and filtered LiDAR plus the transforms required to render/reprocess it.
    "/scan"
    "/scan_filtered"
    "/tf"
    "/tf_static"

    # Sensors and calibration intermediate outputs.
    "/imu"
    "/camera/image_raw/compressed"
    "/camera/pan"
    "/camera/tilt"
    "/joint_states"
    "/calibration/detected_centerline"
    "/calibration/debug/bev/compressed"
    "/calibration/debug/lane_mask/compressed"
    "/calibration/debug/lane_overlay/compressed"

    # Simulation time and commands make the run reproducible.
    "/clock"
    "/speed"
    "/steering"
)

timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="$RECORDS_ROOT/${RUN_LABEL}_${timestamp}"
mkdir -p "$RECORDS_ROOT"

echo
echo "Odometry identity"
echo "  /odom                       fused EKF odometry"
echo "  /odom/laser                 LiDAR-only odometry"
echo "  /odom/calibride             lane-corrected odometry"
echo "  /sim/ground_truth/tf        simulator absolute world truth"
echo "  /sim/ground_truth/odometry  simulator local ground-truth odometry"
echo
echo "Recording calibration diagnostics to: $output_dir"
echo "Drive the complete comparison route, then press Ctrl+C once."

exec ros2 bag record \
    --storage mcap \
    --output "$output_dir" \
    "${topics[@]}"

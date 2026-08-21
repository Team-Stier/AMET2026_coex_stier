#!/usr/bin/env bash

set -Eeuo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

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

declare -A EXPECTED_TYPES=(
    ["/scan"]="sensor_msgs/msg/LaserScan"
    ["/odom"]="nav_msgs/msg/Odometry"
    ["/camera/image_raw/compressed"]="sensor_msgs/msg/CompressedImage"
)

declare -a OPTIONAL_TOPICS=(
    "/scan_filtered"
    "/imu"
    "/odom/laser"
    "/clock"
    "/camera/pan"
    "/tf"
    "/tf_static"
)

topic_table="$(ros2 topic list --show-types)"
missing=0

echo "Required mapping topics"
for topic in "${!EXPECTED_TYPES[@]}"; do
    expected_type="${EXPECTED_TYPES[$topic]}"
    if grep -Fqx "$topic [$expected_type]" <<<"$topic_table"; then
        echo "  OK      $topic [$expected_type]"
    else
        echo "  MISSING $topic [$expected_type]"
        missing=1
    fi
done

echo "Optional mapping topics"
for topic in "${OPTIONAL_TOPICS[@]}"; do
    line="$(grep -F "$topic [" <<<"$topic_table" || true)"
    if [[ -n "$line" ]]; then
        echo "  FOUND   $line"
    else
        echo "  ABSENT  $topic"
    fi
done

if ((missing != 0)); then
    echo "error: required topics are not ready; do not start the mapping run" >&2
    exit 2
fi

echo
echo "Required topics are present. Verify TF and message rates before driving:"
echo "  ros2 run tf2_ros tf2_echo odom base_footprint"
echo "  ros2 topic echo /scan --once --field header"
echo "  ros2 topic hz /scan"
echo "  ros2 topic hz /odom"
echo "  ros2 topic hz /camera/image_raw/compressed"

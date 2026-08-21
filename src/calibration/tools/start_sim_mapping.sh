#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PARAMS_FILE="${SLAM_PARAMS_FILE:-$SCRIPT_DIR/../config/slam_toolbox_sim.yaml}"
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

if [[ ! -f "$PARAMS_FILE" ]]; then
    echo "error: SLAM parameter file not found: $PARAMS_FILE" >&2
    exit 1
fi

if ! ros2 pkg executables slam_toolbox | grep -q 'async_slam_toolbox_node'; then
    echo "error: slam_toolbox async node is not installed" >&2
    exit 1
fi

"$SCRIPT_DIR/check_mapping_topics.sh"

echo "Starting slam_toolbox with: $PARAMS_FILE"
echo "RViz fixed frame must be 'map'. Drive manually and close the loop near the start."

exec ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:="$PARAMS_FILE" \
    use_sim_time:=true

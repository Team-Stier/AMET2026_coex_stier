#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
RECORDS_ROOT="${RECORDS_ROOT:-$WORKSPACE_ROOT/records/mapping}"
RUN_LABEL="${1:-manual}"

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

"$SCRIPT_DIR/check_mapping_topics.sh"

timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="$RECORDS_ROOT/${timestamp}_${RUN_LABEL}"
mkdir -p "$RECORDS_ROOT"

declare -a topics=(
    "/scan"
    "/scan_filtered"
    "/odom"
    "/odom/laser"
    "/imu"
    "/clock"
    "/tf"
    "/tf_static"
)

echo "Recording mapping inputs to: $output_dir"
echo "Drive slowly, revisit the start area, and press Ctrl+C once."

exec ros2 bag record \
    --output "$output_dir" \
    "${topics[@]}"

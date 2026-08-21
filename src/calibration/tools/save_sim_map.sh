#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
MAPS_ROOT="${MAPS_ROOT:-$WORKSPACE_ROOT/maps/occupancy}"
MAP_LABEL="${1:-sim_map}"

if [[ ! "$MAP_LABEL" =~ ^[A-Za-z0-9._-]+$ ]]; then
    echo "error: map label may contain only letters, numbers, dot, underscore, and dash" >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "error: ros2 not found" >&2
    exit 1
fi

if ! ros2 pkg executables nav2_map_server | grep 'map_saver_cli' >/dev/null; then
    echo "error: nav2_map_server map_saver_cli is not installed" >&2
    exit 1
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="$MAPS_ROOT/${timestamp}_${MAP_LABEL}"
map_file="$output_dir/map"
posegraph_file="$output_dir/posegraph"
mkdir -p "$output_dir"

echo "Saving occupancy map to: $map_file"
ros2 run nav2_map_server map_saver_cli \
    -f "$map_file" \
    --ros-args -p use_sim_time:=true

if ros2 service type /slam_toolbox/serialize_map 2>/dev/null \
    | grep -q 'slam_toolbox/srv/SerializePoseGraph'; then
    echo "Saving slam_toolbox pose graph to: $posegraph_file"
    ros2 service call \
        /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph \
        "{filename: '$posegraph_file'}"
else
    echo "warning: /slam_toolbox/serialize_map is unavailable; occupancy map only" >&2
fi

echo "Saved mapping artifacts under: $output_dir"

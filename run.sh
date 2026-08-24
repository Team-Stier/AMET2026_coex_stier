#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    bash "${BASH_SOURCE[0]}" "$@"
    return $?
fi

set -Eeuo pipefail

WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
BRINGUP_DELAY_SEC="${BRINGUP_DELAY_SEC:-0.5}"

if [[ -f "$ROS_SETUP" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "$ROS_SETUP"
    set -u
elif [[ "${ROS_DISTRO:-}" != "jazzy" ]]; then
    echo "error: ROS 2 Jazzy setup not found: $ROS_SETUP" >&2
    echo "set ROS_SETUP to the Jazzy setup.bash path and try again" >&2
    exit 1
fi

cd "$WORKSPACE_ROOT"
colcon build --cmake-clean-cache

# shellcheck disable=SC1091
set +u
source "$WORKSPACE_ROOT/install/setup.bash"
set -u

declare -a PIDS=()
declare -a NODE_NAMES=()
declare -a NODE_MODES=()

cleanup() {
    local exit_code=$?
    trap - EXIT INT TERM

    if ((${#PIDS[@]} > 0)); then
        kill -TERM "${PIDS[@]}" 2>/dev/null || true
        for pid in "${PIDS[@]}"; do
            wait "$pid" 2>/dev/null || true
        done
    fi

    exit "$exit_code"
}

trap cleanup EXIT INT TERM

start_node() {
    local package_name=$1
    local executable=$2
    shift 2

    local mode=${1:-persistent}
    if (($# > 0)); then
        shift
    fi

    echo "[bringup] starting $package_name/$executable"
    ros2 run "$package_name" "$executable" "$@" &
    PIDS+=("$!")
    NODE_NAMES+=("$package_name/$executable")
    NODE_MODES+=("$mode")
    sleep "$BRINGUP_DELAY_SEC"
}

start_node "object_detection" "object_detection_node"
start_node "control" "control_node"
start_node "traffic_light" "traffic_light_node" "oneshot"
start_node "pose_tf" "pose_tf_node"
start_node "path_planning" "path_planning_node" "persistent" \
    --ros-args --params-file \
    "${WORKSPACE_ROOT}/install/path_planning/share/path_planning/config/path_planning.yaml"

echo "[bringup] starting visualizer/launch.sh"
"${WORKSPACE_ROOT}/src/visualizer/launch.sh" &
PIDS+=("$!")
NODE_NAMES+=("visualizer/launch.sh")
NODE_MODES+=("persistent")

echo "[bringup] all nodes started"

while ((${#PIDS[@]} > 0)); do
    for index in "${!PIDS[@]}"; do
        pid=${PIDS[$index]}
        if kill -0 "$pid" 2>/dev/null; then
            continue
        fi

        set +e
        wait "$pid"
        status=$?
        set -e

        name=${NODE_NAMES[$index]}
        mode=${NODE_MODES[$index]}
        unset 'PIDS[index]' 'NODE_NAMES[index]' 'NODE_MODES[index]'

        if [[ "$mode" == "oneshot" && "$status" -eq 0 ]]; then
            echo "[bringup] one-shot node completed: $name"
            continue
        fi

        echo "error: node stopped unexpectedly: $name (exit $status)" >&2
        if [[ "$status" -eq 0 ]]; then
            exit 1
        fi
        exit "$status"
    done

    sleep 1
done

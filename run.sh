#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    bash "${BASH_SOURCE[0]}" "$@"
    return $?
fi

set -Eeuo pipefail
set +m

WORKSPACE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
BRINGUP_DELAY_SEC="${BRINGUP_DELAY_SEC:-0.5}"
RUNTIME_KEY="$(printf '%s' "$WORKSPACE_ROOT" | cksum | awk '{print $1}')"
RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}/physicar-run-${UID}-${RUNTIME_KEY}"
RUN_STATE_FILE="${RUNTIME_DIR}/run-id"
RUN_LOCK_FILE="${RUNTIME_DIR}/lock"
LEGACY_PROCESS_PATTERN="${WORKSPACE_ROOT}/install/(object_detection|control|traffic_light|pose_tf|calibration|path_planning|visualizer)/"

mkdir -p -m 700 "$RUNTIME_DIR"
exec {RUN_LOCK_FD}>"$RUN_LOCK_FILE"
if ! flock -n "$RUN_LOCK_FD"; then
    echo "error: another run.sh is already active for $WORKSPACE_ROOT" >&2
    exit 1
fi

process_has_run_id() {
    local pid=$1
    local run_id=$2
    local entry

    while IFS= read -r -d '' entry; do
        [[ "$entry" == "PHYSICAR_RUN_ID=$run_id" ]] && return 0
    done 2>/dev/null <"/proc/$pid/environ"
    return 1
}

groups_for_run() {
    local run_id=$1
    local pid pgid
    declare -A seen=()

    while read -r pid pgid; do
        if process_has_run_id "$pid" "$run_id" && [[ -z "${seen[$pgid]:-}" ]]; then
            seen[$pgid]=1
            printf '%s\n' "$pgid"
        fi
    done < <(ps -eo pid=,pgid=)
}

run_is_alive() {
    [[ -n "$(groups_for_run "$1")" ]]
}

signal_run() {
    local run_id=$1
    local signal=$2
    local pgid

    while read -r pgid; do
        [[ -n "$pgid" ]] && kill -s "$signal" -- "-$pgid" 2>/dev/null || true
    done < <(groups_for_run "$run_id")
}

wait_for_run() {
    local run_id=$1
    local deadline=$((SECONDS + $2))

    while ((SECONDS < deadline)); do
        run_is_alive "$run_id" || return 0
        sleep 0.1
    done
    ! run_is_alive "$run_id"
}

stop_run() {
    local run_id=$1

    run_is_alive "$run_id" || return 0
    signal_run "$run_id" INT
    wait_for_run "$run_id" 5 && return 0
    signal_run "$run_id" TERM
    wait_for_run "$run_id" 2 && return 0
    signal_run "$run_id" KILL
    wait_for_run "$run_id" 1
}

legacy_pids() {
    pgrep -u "$UID" -f -- "$LEGACY_PROCESS_PATTERN" 2>/dev/null || true
}

legacy_is_alive() {
    [[ -n "$(legacy_pids)" ]]
}

signal_legacy() {
    local signal=$1
    local pid

    while read -r pid; do
        [[ -n "$pid" ]] && kill -s "$signal" "$pid" 2>/dev/null || true
    done < <(legacy_pids)
}

wait_for_legacy() {
    local deadline=$((SECONDS + $1))

    while ((SECONDS < deadline)); do
        legacy_is_alive || return 0
        sleep 0.1
    done
    ! legacy_is_alive
}

stop_legacy() {
    legacy_is_alive || return 0
    signal_legacy INT
    wait_for_legacy 5 && return 0
    signal_legacy TERM
    wait_for_legacy 2 && return 0
    signal_legacy KILL
    wait_for_legacy 1
}

if [[ -s "$RUN_STATE_FILE" ]]; then
    read -r STALE_RUN_ID <"$RUN_STATE_FILE"
    if run_is_alive "$STALE_RUN_ID"; then
        echo "[bringup] cleaning processes left by the previous run"
        if ! stop_run "$STALE_RUN_ID"; then
            echo "error: failed to stop processes left by the previous run" >&2
            exit 1
        fi
    fi
    rm -f "$RUN_STATE_FILE"
fi

if legacy_is_alive; then
    echo "[bringup] cleaning legacy workspace processes"
    if ! stop_legacy; then
        echo "error: failed to stop legacy workspace processes" >&2
        exit 1
    fi
fi

RUN_ID="${BASHPID}-$(date +%s%N)"
printf '%s\n' "$RUN_ID" >"$RUN_STATE_FILE"

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

ros2 daemon stop >/dev/null 2>&1 || true

cd "$WORKSPACE_ROOT"
colcon build --cmake-clean-cache

# shellcheck disable=SC1091
set +u
source "$WORKSPACE_ROOT/install/setup.bash"
set -u

declare -a PIDS=()
declare -a NODE_NAMES=()
declare -a NODE_MODES=()

stop_current_run() {
    local index pid

    run_is_alive "$RUN_ID" || return 0
    for index in "${!PIDS[@]}"; do
        pid=${PIDS[$index]}
        if [[ "${NODE_NAMES[$index]}" == "visualizer/launch.sh" ]]; then
            kill -s INT "$pid" 2>/dev/null || true
        else
            kill -s INT -- "-$pid" 2>/dev/null || true
        fi
    done
    wait_for_run "$RUN_ID" 5 && return 0
    signal_run "$RUN_ID" TERM
    wait_for_run "$RUN_ID" 2 && return 0
    signal_run "$RUN_ID" KILL
    wait_for_run "$RUN_ID" 1
}

cleanup() {
    local exit_code=$?
    trap - EXIT
    trap '' INT TERM

    stop_current_run || echo "warning: some run.sh processes could not be stopped" >&2

    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done

    if ! run_is_alive "$RUN_ID"; then
        rm -f "$RUN_STATE_FILE"
    fi

    exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_node() {
    local package_name=$1
    local executable=$2
    shift 2

    local mode=${1:-persistent}
    if (($# > 0)); then
        shift
    fi

    echo "[bringup] starting $package_name/$executable"
    setsid env --default-signal=INT --default-signal=TERM --default-signal=QUIT \
        PHYSICAR_RUN_ID="$RUN_ID" ros2 run "$package_name" "$executable" "$@" \
        {RUN_LOCK_FD}>&- &
    PIDS+=("$!")
    NODE_NAMES+=("$package_name/$executable")
    NODE_MODES+=("$mode")
    sleep "$BRINGUP_DELAY_SEC"
}

start_node "object_detection" "object_detection_node"
start_node "control" "control_node" "persistent" \
    --ros-args --params-file \
    "${WORKSPACE_ROOT}/install/control/share/control/config/control.yaml"
start_node "traffic_light" "traffic_light_node" "oneshot"
start_node "pose_tf" "pose_tf_node" "persistent" \
    --ros-args --params-file \
    "${WORKSPACE_ROOT}/install/pose_tf/share/pose_tf/config/pose_tf.yaml"
start_node "calibration" "calibration_node"
start_node "path_planning" "path_planning_node" "persistent" \
    --ros-args --params-file \
    "${WORKSPACE_ROOT}/install/path_planning/share/path_planning/config/path_planning.yaml"

echo "[bringup] starting visualizer/launch.sh"
setsid env --default-signal=INT --default-signal=TERM --default-signal=QUIT \
    PHYSICAR_RUN_ID="$RUN_ID" "${WORKSPACE_ROOT}/src/visualizer/launch.sh" \
    {RUN_LOCK_FD}>&- &
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

        if [[ "$mode" == "oneshot" && "$status" -eq 0 ]]; then
            unset 'PIDS[index]' 'NODE_NAMES[index]' 'NODE_MODES[index]'
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

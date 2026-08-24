#!/usr/bin/env bash
set -e

PACKAGE_SHARE="$(ros2 pkg prefix --share path_planning)"
exec ros2 run path_planning path_planning_node --ros-args --params-file "${PACKAGE_SHARE}/config/path_planning.yaml"

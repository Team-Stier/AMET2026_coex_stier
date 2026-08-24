#!/usr/bin/env bash

set -Eeuo pipefail

WORLD_NAME="${SIM_WORLD_NAME:-custom_e09090b056ef1f90f845419690065271}"
REMOVE_SERVICE="/world/$WORLD_NAME/remove"
declare -a CONE_MODELS=(cone1 cone2 cone3 cone5 cone7 cone8)

if ! command -v gz >/dev/null 2>&1; then
    echo "error: Gazebo CLI 'gz' is not available" >&2
    exit 1
fi

if ! gz service -l | grep -Fqx "$REMOVE_SERVICE"; then
    echo "error: Gazebo remove service is unavailable: $REMOVE_SERVICE" >&2
    exit 1
fi

for model_name in "${CONE_MODELS[@]}"; do
    response="$(
        gz service \
            -s "$REMOVE_SERVICE" \
            --reqtype gz.msgs.Entity \
            --reptype gz.msgs.Boolean \
            --timeout 2000 \
            --req "name: \"$model_name\", type: MODEL"
    )"
    if grep -Fq "data: true" <<<"$response"; then
        echo "removed $model_name"
    else
        echo "error: Gazebo did not remove $model_name: $response" >&2
        exit 2
    fi
done

model_list="$(gz model --list 2>&1)"
for model_name in "${CONE_MODELS[@]}"; do
    if grep -Eq "^[[:space:]]*-[[:space:]]+$model_name$" <<<"$model_list"; then
        echo "error: $model_name is still present after removal" >&2
        exit 3
    fi
done

echo "All mapping cones are absent from Gazebo. Start SLAM before resetting the simulator."

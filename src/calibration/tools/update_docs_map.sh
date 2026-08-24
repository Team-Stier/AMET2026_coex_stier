#!/usr/bin/env bash

set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
DEFAULT_MAP_DIR="$WORKSPACE_ROOT/maps/occupancy/20260820_062838_conefree_auto_control_slam"
SOURCE_IMAGE="${1:-$DEFAULT_MAP_DIR/camera_fusion/lidar_camera_fused.png}"
TARGET_IMAGE="$WORKSPACE_ROOT/src/calibration/docs/map.png"

if [[ ! -s "$SOURCE_IMAGE" ]]; then
    echo "error: source image is missing or empty: $SOURCE_IMAGE" >&2
    exit 1
fi

if command -v file >/dev/null 2>&1; then
    mime_type="$(file --brief --mime-type "$SOURCE_IMAGE")"
    if [[ "$mime_type" != "image/png" ]]; then
        echo "error: source is not a PNG image: $SOURCE_IMAGE ($mime_type)" >&2
        exit 1
    fi
fi

mkdir -p "$(dirname -- "$TARGET_IMAGE")"
temporary_image="$(mktemp "${TARGET_IMAGE}.tmp.XXXXXX")"

cleanup() {
    rm -f -- "$temporary_image"
}
trap cleanup EXIT

cp -- "$SOURCE_IMAGE" "$temporary_image"
chmod 0644 "$temporary_image"
mv -f -- "$temporary_image" "$TARGET_IMAGE"
trap - EXIT

echo "Updated documentation map: $TARGET_IMAGE"
file "$TARGET_IMAGE" 2>/dev/null || true
ls -lh "$TARGET_IMAGE"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_PROFILE=autodrive IMAGE_SCRIPT_NAME=./scripts/shared/build_vehicle_autodrive_images.sh exec "${SCRIPT_DIR}/lib/build_vehicle_image_profiles.sh" "$@"

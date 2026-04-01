#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE_PROFILE=autodrive exec "${SCRIPT_DIR}/build_vehicle_orchestration_images.sh" "$@"

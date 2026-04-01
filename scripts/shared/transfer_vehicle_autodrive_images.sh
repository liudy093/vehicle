#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/vehicle_inventory.sh"

VEHICLE_HOSTS="${VEHICLE_HOSTS:-$(vehicle_inventory_host "vehicle1" "nvidia") $(vehicle_inventory_host "vehicle2" "nvidia") $(vehicle_inventory_host "vehicle3" "nvidia")}"

IMAGE_PROFILE=autodrive IMAGE_SCRIPT_NAME=./scripts/shared/transfer_vehicle_autodrive_images.sh VEHICLE_HOSTS="${VEHICLE_HOSTS}" exec "${SCRIPT_DIR}/lib/transfer_vehicle_image_profiles.sh" "$@"

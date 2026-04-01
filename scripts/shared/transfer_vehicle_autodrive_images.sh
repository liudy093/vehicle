#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/vehicle_inventory.sh"

VEHICLE_HOSTS="${VEHICLE_HOSTS:-$(vehicle_inventory_host "vehicle1" "nvidia") $(vehicle_inventory_host "vehicle2" "nvidia") $(vehicle_inventory_host "vehicle3" "nvidia")}"

IMAGE_PROFILE=autodrive VEHICLE_HOSTS="${VEHICLE_HOSTS}" exec "${SCRIPT_DIR}/transfer_vehicle_orchestration_images.sh" "$@"

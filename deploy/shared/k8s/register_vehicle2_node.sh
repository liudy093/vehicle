#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../../../scripts/shared/vehicle_inventory.sh"

VEHICLE_ID="${VEHICLE_ID:-vehicle2}"
DEFAULT_VEHICLE_HOST="$(vehicle_inventory_host "vehicle2" "nvidia")"
VEHICLE_HOST="${VEHICLE_HOST:-${DEFAULT_VEHICLE_HOST}}"
VEHICLE_ID="${VEHICLE_ID}" VEHICLE_HOST="${VEHICLE_HOST}" exec "${SCRIPT_DIR}/register_vehicle_node.sh"

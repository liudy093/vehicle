#!/usr/bin/env bash
set -euo pipefail

# Edit these values for car 1 before first use.
export VEHICLE_ID="${VEHICLE_ID:-vehicle1}"
export CLOUD_SERVER_URL="${CLOUD_SERVER_URL:-http://127.0.0.1:5000}"
export GPS_SERIAL_PORT="${GPS_SERIAL_PORT:-/dev/ttyUSB1}"
export GPS_LAT0="${GPS_LAT0:-39.95754418}"
export GPS_LON0="${GPS_LON0:-116.30460525}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run-car.sh" "$@"

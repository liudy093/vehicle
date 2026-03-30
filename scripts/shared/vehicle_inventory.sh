#!/usr/bin/env bash

VEHICLE_INVENTORY_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VEHICLE_INVENTORY_FILE="${VEHICLE_INVENTORY_FILE:-${VEHICLE_INVENTORY_SCRIPT_DIR}/../../config/shared/vehicle_hosts.env}"

if [[ -f "${VEHICLE_INVENTORY_FILE}" ]]; then
  # shellcheck disable=SC1090
  source "${VEHICLE_INVENTORY_FILE}"
fi

vehicle_inventory_host() {
  local vehicle_id="${1:?vehicle id is required}"
  local account_kind="${2:-nvidia}"
  local key=""

  case "${vehicle_id}:${account_kind}" in
    vehicle1:nvidia) key="VEHICLE1_HOST" ;;
    vehicle2:nvidia) key="VEHICLE2_HOST" ;;
    vehicle1:root) key="VEHICLE1_ROOT_HOST" ;;
    vehicle2:root) key="VEHICLE2_ROOT_HOST" ;;
    *)
      echo "Unsupported vehicle inventory lookup: ${vehicle_id}/${account_kind}" >&2
      return 1
      ;;
  esac

  if [[ -z "${!key:-}" ]]; then
    echo "Missing ${key} in ${VEHICLE_INVENTORY_FILE}" >&2
    return 1
  fi

  printf '%s\n' "${!key}"
}

vehicle_inventory_expected_hostname() {
  local vehicle_id="${1:?vehicle id is required}"
  local key=""

  case "${vehicle_id}" in
    vehicle1) key="VEHICLE1_EXPECTED_HOSTNAME" ;;
    vehicle2) key="VEHICLE2_EXPECTED_HOSTNAME" ;;
    *)
      echo "Unsupported vehicle hostname lookup: ${vehicle_id}" >&2
      return 1
      ;;
  esac

  if [[ -n "${!key:-}" ]]; then
    printf '%s\n' "${!key}"
    return 0
  fi

  printf '%s\n' "${vehicle_id}"
}

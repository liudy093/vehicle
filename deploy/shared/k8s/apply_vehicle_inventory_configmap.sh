#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/vehicle.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
INVENTORY_FILE="${VEHICLE_INVENTORY_FILE:-${ROOT_DIR}/config/shared/vehicle_hosts.env}"
NAMESPACE="${NAMESPACE:-argo}"
CONFIGMAP_NAME="${CONFIGMAP_NAME:-vehicle-inventory}"

if ! command -v kubectl >/dev/null 2>&1; then
  echo "kubectl is required" >&2
  exit 1
fi

if [[ ! -f "${INVENTORY_FILE}" ]]; then
  echo "Inventory file not found: ${INVENTORY_FILE}" >&2
  exit 1
fi

kubectl -n "${NAMESPACE}" create configmap "${CONFIGMAP_NAME}" \
  --from-file=vehicle_hosts.env="${INVENTORY_FILE}" \
  --dry-run=client \
  -o yaml | kubectl apply -f -

echo "Applied ${CONFIGMAP_NAME} in namespace ${NAMESPACE} from ${INVENTORY_FILE}."

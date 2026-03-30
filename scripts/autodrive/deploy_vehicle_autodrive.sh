#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/perception.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
K8S_DIR="${ROOT_DIR}/deploy/shared/k8s"
ARGO_DIR="${ROOT_DIR}/deploy/autodrive/argo"
source "${SCRIPT_DIR}/../shared/lib/common.sh"
source "${SCRIPT_DIR}/../shared/vehicle_inventory.sh"

WORKFLOW_FILE="${WORKFLOW_FILE:-${ARGO_DIR}/vehicle-autodrive-workflow.yaml}"
BOOTSTRAP_CLUSTER="${BOOTSTRAP_CLUSTER:-false}"
TARGET_VEHICLE="${TARGET_VEHICLE:-}"
AUTODRIVE_RUNNER_IMAGE="${AUTODRIVE_RUNNER_IMAGE:-vehicle-autodrive-runner:latest}"
DEFAULT_VEHICLE1_HOST="$(vehicle_inventory_host "vehicle1" "nvidia")"
DEFAULT_VEHICLE2_HOST="$(vehicle_inventory_host "vehicle2" "nvidia")"
DEFAULT_VEHICLE1_HOSTNAME="$(vehicle_inventory_expected_hostname "vehicle1")"
DEFAULT_VEHICLE2_HOSTNAME="$(vehicle_inventory_expected_hostname "vehicle2")"

usage() {
  cat <<EOF
Usage:
  TARGET_VEHICLE=vehicle1 ./scripts/autodrive/deploy_vehicle_autodrive.sh

Environment variables:
  WORKFLOW_FILE              Default: ${WORKFLOW_FILE}
  BOOTSTRAP_CLUSTER          Default: ${BOOTSTRAP_CLUSTER}
  TARGET_VEHICLE             Required: vehicle1 or vehicle2
  AUTODRIVE_RUNNER_IMAGE     Default: ${AUTODRIVE_RUNNER_IMAGE}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

case "${TARGET_VEHICLE}" in
  vehicle1|vehicle2)
    ;;
  *)
    echo "TARGET_VEHICLE must be set to vehicle1 or vehicle2" >&2
    usage >&2
    exit 1
    ;;
esac

vehicle_node_ready() {
  local node="$1"
  local ready_status=""

  if ! kubectl get node "${node}" >/dev/null 2>&1; then
    return 1
  fi

  ready_status="$(kubectl get node "${node}" -o jsonpath='{range .status.conditions[?(@.type=="Ready")]}{.status}{end}')"
  [[ "${ready_status}" == "True" ]]
}

render_targeted_workflow() {
  local source_workflow="$1"
  local rendered_workflow="$2"
  local target_vehicle="$3"
  local autodrive_runner_image="$4"

  TARGET_VEHICLE="${target_vehicle}" AUTODRIVE_RUNNER_IMAGE="${autodrive_runner_image}" python3 - "${source_workflow}" "${rendered_workflow}" <<'PY'
import pathlib
import os
import sys


source_path = pathlib.Path(sys.argv[1])
rendered_path = pathlib.Path(sys.argv[2])
autodrive_runner_image = os.environ["AUTODRIVE_RUNNER_IMAGE"]
target_vehicle = os.environ["TARGET_VEHICLE"]

rendered_path.write_text(
    source_path.read_text()
    .replace("${AUTODRIVE_RUNNER_IMAGE}", autodrive_runner_image)
    .replace("${TARGET_VEHICLE}", target_vehicle)
)
PY
}

for cmd in bash kubectl python3; do
  require_cmd "${cmd}"
done

echo "Using vehicle inventory:"
echo "  vehicle1 -> ${DEFAULT_VEHICLE1_HOST}"
echo "  vehicle2 -> ${DEFAULT_VEHICLE2_HOST}"
echo "Configured vehicle hostnames:"
echo "  vehicle1 -> ${DEFAULT_VEHICLE1_HOSTNAME}"
echo "  vehicle2 -> ${DEFAULT_VEHICLE2_HOSTNAME}"
echo "Target vehicle: ${TARGET_VEHICLE}"

if [[ "${BOOTSTRAP_CLUSTER}" == "true" ]]; then
  echo "Bootstrapping cluster infrastructure and vehicle agents..."
  bash "${K8S_DIR}/argo-install.sh"
  bash "${K8S_DIR}/register_vehicle1_node.sh"
  bash "${K8S_DIR}/register_vehicle2_node.sh"
else
  if ! vehicle_node_ready "${TARGET_VEHICLE}"; then
    echo "Target vehicle node ${TARGET_VEHICLE} is not Ready." >&2
    exit 1
  fi

  echo "Skipping Argo installation and vehicle agent registration. Assuming cluster infrastructure and vehicle nodes already exist."
  APPLY_ARGO_INSTALL_MANIFEST=false bash "${K8S_DIR}/argo-install.sh"
fi

bash "${K8S_DIR}/register_general_workers.sh"
bash "${K8S_DIR}/apply_vehicle_inventory_configmap.sh"

if ! vehicle_node_ready "${TARGET_VEHICLE}"; then
  echo "Target vehicle node ${TARGET_VEHICLE} is not Ready." >&2
  exit 1
fi

kubectl apply -f "${K8S_DIR}/namespace-perception.yaml"

rendered_workflow_file="$(mktemp "${TMPDIR:-/tmp}/vehicle-autodrive-rendered.XXXXXX.yaml")"
trap 'rm -f "${rendered_workflow_file}"' EXIT
render_targeted_workflow "${WORKFLOW_FILE}" "${rendered_workflow_file}" "${TARGET_VEHICLE}" "${AUTODRIVE_RUNNER_IMAGE}"

workflow_name="$(kubectl create -f "${rendered_workflow_file}" -o name)"
echo "Submitted ${workflow_name}"
echo "Watch progress with:"
echo "  kubectl get wf -n argo"
echo "  kubectl get pods -n perception"

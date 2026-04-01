#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/perception.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
K8S_DIR="${ROOT_DIR}/deploy/shared/k8s"
ARGO_DIR="${ROOT_DIR}/deploy/autodrive/argo"
source "${SCRIPT_DIR}/../shared/lib/common.sh"
source "${SCRIPT_DIR}/../shared/lib/vehicle_autodrive_common.sh"
source "${SCRIPT_DIR}/../shared/vehicle_inventory.sh"

WORKFLOW_FILE="${WORKFLOW_FILE:-${ARGO_DIR}/vehicle-autodrive-workflow.yaml}"
BOOTSTRAP_CLUSTER="${BOOTSTRAP_CLUSTER:-false}"
AUTODRIVE_RUNNER_IMAGE="${AUTODRIVE_RUNNER_IMAGE:-vehicle-autodrive-runner:latest}"
VEHICLE1_ENDPOINT="${VEHICLE1_ENDPOINT:-end_1}"
VEHICLE2_ENDPOINT="${VEHICLE2_ENDPOINT:-end_2}"
VEHICLE3_ENDPOINT="${VEHICLE3_ENDPOINT:-end_3}"
DEFAULT_VEHICLE1_HOST="$(vehicle_inventory_host "vehicle1" "nvidia")"
DEFAULT_VEHICLE2_HOST="$(vehicle_inventory_host "vehicle2" "nvidia")"
DEFAULT_VEHICLE3_HOST="$(vehicle_inventory_host "vehicle3" "nvidia")"
DEFAULT_VEHICLE1_HOSTNAME="$(vehicle_inventory_expected_hostname "vehicle1")"
DEFAULT_VEHICLE2_HOSTNAME="$(vehicle_inventory_expected_hostname "vehicle2")"
DEFAULT_VEHICLE3_HOSTNAME="$(vehicle_inventory_expected_hostname "vehicle3")"

usage() {
  cat <<EOF
Usage:
  ./scripts/autodrive/deploy_triple_vehicle_autodrive.sh

Description:
  Submit three autodrive workflows together:
  - vehicle1 -> ${VEHICLE1_ENDPOINT}
  - vehicle2 -> ${VEHICLE2_ENDPOINT}
  - vehicle3 -> ${VEHICLE3_ENDPOINT}

Safety:
  Do not run this script unless personnel are on the vehicles.

Environment variables:
  WORKFLOW_FILE              Default: ${WORKFLOW_FILE}
  BOOTSTRAP_CLUSTER          Default: ${BOOTSTRAP_CLUSTER}
  AUTODRIVE_RUNNER_IMAGE     Default: ${AUTODRIVE_RUNNER_IMAGE}
  VEHICLE1_ENDPOINT          Default: ${VEHICLE1_ENDPOINT} (end_1, end_2, or end_3)
  VEHICLE2_ENDPOINT          Default: ${VEHICLE2_ENDPOINT} (end_1, end_2, or end_3)
  VEHICLE3_ENDPOINT          Default: ${VEHICLE3_ENDPOINT} (end_1, end_2, or end_3)
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if ! vehicle_autodrive_endpoint_coords "${VEHICLE1_ENDPOINT}" >/dev/null; then
  usage >&2
  exit 1
fi

if ! vehicle_autodrive_endpoint_coords "${VEHICLE2_ENDPOINT}" >/dev/null; then
  usage >&2
  exit 1
fi

if ! vehicle_autodrive_endpoint_coords "${VEHICLE3_ENDPOINT}" >/dev/null; then
  usage >&2
  exit 1
fi

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
  local target_endpoint="$4"
  local autodrive_runner_image="$5"

  TARGET_VEHICLE="${target_vehicle}" TARGET_ENDPOINT="${target_endpoint}" AUTODRIVE_RUNNER_IMAGE="${autodrive_runner_image}" python3 - "${source_workflow}" "${rendered_workflow}" <<'PY'
import pathlib
import os
import sys


source_path = pathlib.Path(sys.argv[1])
rendered_path = pathlib.Path(sys.argv[2])
autodrive_runner_image = os.environ["AUTODRIVE_RUNNER_IMAGE"]
target_vehicle = os.environ["TARGET_VEHICLE"]
target_endpoint = os.environ["TARGET_ENDPOINT"]

rendered_path.write_text(
    source_path.read_text()
    .replace("${AUTODRIVE_RUNNER_IMAGE}", autodrive_runner_image)
    .replace("${TARGET_ENDPOINT}", target_endpoint)
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
echo "  vehicle3 -> ${DEFAULT_VEHICLE3_HOST}"
echo "Configured vehicle hostnames:"
echo "  vehicle1 -> ${DEFAULT_VEHICLE1_HOSTNAME}"
echo "  vehicle2 -> ${DEFAULT_VEHICLE2_HOSTNAME}"
echo "  vehicle3 -> ${DEFAULT_VEHICLE3_HOSTNAME}"
echo "Target endpoints:"
echo "  vehicle1 -> ${VEHICLE1_ENDPOINT}"
echo "  vehicle2 -> ${VEHICLE2_ENDPOINT}"
echo "  vehicle3 -> ${VEHICLE3_ENDPOINT}"
echo "Safety reminder: ensure personnel are on all vehicles before running."

if [[ "${BOOTSTRAP_CLUSTER}" == "true" ]]; then
  echo "Bootstrapping cluster infrastructure and vehicle agents..."
  bash "${K8S_DIR}/argo-install.sh"
  VEHICLE_ID="vehicle1" bash "${K8S_DIR}/register_vehicle_node.sh"
  VEHICLE_ID="vehicle2" bash "${K8S_DIR}/register_vehicle_node.sh"
  VEHICLE_ID="vehicle3" bash "${K8S_DIR}/register_vehicle_node.sh"
else
  if ! vehicle_node_ready "vehicle1"; then
    echo "Target vehicle node vehicle1 is not Ready." >&2
    exit 1
  fi

  if ! vehicle_node_ready "vehicle2"; then
    echo "Target vehicle node vehicle2 is not Ready." >&2
    exit 1
  fi

  if ! vehicle_node_ready "vehicle3"; then
    echo "Target vehicle node vehicle3 is not Ready." >&2
    exit 1
  fi

  echo "Skipping Argo installation and vehicle agent registration. Assuming cluster infrastructure and vehicle nodes already exist."
  APPLY_ARGO_INSTALL_MANIFEST=false bash "${K8S_DIR}/argo-install.sh"
fi

bash "${K8S_DIR}/register_general_workers.sh"
bash "${K8S_DIR}/apply_vehicle_inventory_configmap.sh"

if ! vehicle_node_ready "vehicle1"; then
  echo "Target vehicle node vehicle1 is not Ready." >&2
  exit 1
fi

if ! vehicle_node_ready "vehicle2"; then
  echo "Target vehicle node vehicle2 is not Ready." >&2
  exit 1
fi

if ! vehicle_node_ready "vehicle3"; then
  echo "Target vehicle node vehicle3 is not Ready." >&2
  exit 1
fi

kubectl apply -f "${K8S_DIR}/namespace-perception.yaml"

rendered_workflow_vehicle1="$(mktemp "${TMPDIR:-/tmp}/vehicle1-autodrive-rendered.XXXXXX.yaml")"
rendered_workflow_vehicle2="$(mktemp "${TMPDIR:-/tmp}/vehicle2-autodrive-rendered.XXXXXX.yaml")"
rendered_workflow_vehicle3="$(mktemp "${TMPDIR:-/tmp}/vehicle3-autodrive-rendered.XXXXXX.yaml")"
trap 'rm -f "${rendered_workflow_vehicle1}" "${rendered_workflow_vehicle2}" "${rendered_workflow_vehicle3}"' EXIT

render_targeted_workflow "${WORKFLOW_FILE}" "${rendered_workflow_vehicle1}" "vehicle1" "${VEHICLE1_ENDPOINT}" "${AUTODRIVE_RUNNER_IMAGE}"
render_targeted_workflow "${WORKFLOW_FILE}" "${rendered_workflow_vehicle2}" "vehicle2" "${VEHICLE2_ENDPOINT}" "${AUTODRIVE_RUNNER_IMAGE}"
render_targeted_workflow "${WORKFLOW_FILE}" "${rendered_workflow_vehicle3}" "vehicle3" "${VEHICLE3_ENDPOINT}" "${AUTODRIVE_RUNNER_IMAGE}"

workflow_name_vehicle1="$(kubectl create -f "${rendered_workflow_vehicle1}" -o name)"
workflow_name_vehicle2="$(kubectl create -f "${rendered_workflow_vehicle2}" -o name)"
workflow_name_vehicle3="$(kubectl create -f "${rendered_workflow_vehicle3}" -o name)"

echo "Submitted ${workflow_name_vehicle1}"
echo "Submitted ${workflow_name_vehicle2}"
echo "Submitted ${workflow_name_vehicle3}"
echo "Watch progress with:"
echo "  kubectl get wf -n argo"
echo "  kubectl get pods -n argo"

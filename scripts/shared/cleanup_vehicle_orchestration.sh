#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/perception.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
K8S_DIR="${ROOT_DIR}/deploy/shared/k8s"
source "${SCRIPT_DIR}/lib/common.sh"
source "${SCRIPT_DIR}/lib/vehicle_remote_runtime.sh"
source "${SCRIPT_DIR}/vehicle_inventory.sh"

ARGO_NAMESPACE="${ARGO_NAMESPACE:-argo}"
PERCEPTION_NAMESPACE="${PERCEPTION_NAMESPACE:-perception}"
ARGO_WORKFLOWS_VERSION="${ARGO_WORKFLOWS_VERSION:-v3.7.12}"
ARGO_INSTALL_MANIFEST_URL="${ARGO_INSTALL_MANIFEST_URL:-https://github.com/argoproj/argo-workflows/releases/download/${ARGO_WORKFLOWS_VERSION}/install.yaml}"
GENERAL_WORKERS="${GENERAL_WORKERS:-k3s-worker-1 k3s-worker-2 k3s-worker-3}"
REMOVE_ARGO="${REMOVE_ARGO:-true}"
REMOVE_CLUSTER_INFRA="${REMOVE_CLUSTER_INFRA:-false}"
VEHICLE1_HOST="${VEHICLE1_HOST:-$(vehicle_inventory_host "vehicle1" "nvidia")}"
VEHICLE2_HOST="${VEHICLE2_HOST:-$(vehicle_inventory_host "vehicle2" "nvidia")}"
SSH_IDENTITY_FILE="${SSH_IDENTITY_FILE:-}"
SSH_OPTS=()
while IFS= read -r opt; do
  SSH_OPTS+=("${opt}")
done < <(build_ssh_opts "${SSH_IDENTITY_FILE}")
REMOTE_RUNTIME_FUNCTIONS="$(vehicle_remote_runtime_functions)"

usage() {
  cat <<EOF
Usage:
  ./scripts/shared/cleanup_vehicle_orchestration.sh

Environment variables:
  ARGO_NAMESPACE       Default: ${ARGO_NAMESPACE}
  PERCEPTION_NAMESPACE Default: ${PERCEPTION_NAMESPACE}
  REMOVE_CLUSTER_INFRA Default: ${REMOVE_CLUSTER_INFRA}
  REMOVE_ARGO          Default: ${REMOVE_ARGO}, only used when REMOVE_CLUSTER_INFRA=true

Description:
  Cleanup only the mapping workflow side of the orchestration stack.
  This stops mapping-related vehicle processes and removes mapping/cloud resources.
  It does not stop autodrive processes.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

for cmd in bash kubectl ssh; do
  require_cmd "${cmd}"
done

if [[ "${REMOVE_CLUSTER_INFRA}" == "true" && "${REMOVE_ARGO}" == "true" ]]; then
  require_cmd curl
fi

stop_vehicle_mapping() {
  local host="$1"
  ssh "${SSH_OPTS[@]}" "${host}" "REMOTE_RUNTIME_FUNCTIONS=$(printf '%q' "${REMOTE_RUNTIME_FUNCTIONS}") bash -s" <<'EOF'
set -euo pipefail
eval "${REMOTE_RUNTIME_FUNCTIONS}"
vehicle_remote_stop_mapping_processes
EOF
}

echo "Stopping vehicle mapping processes..."
stop_vehicle_mapping "${VEHICLE1_HOST}" || true
stop_vehicle_mapping "${VEHICLE2_HOST}" || true

echo "Deleting active mapping workflows and cloud resources..."
mapping_workflows="$(
  kubectl get wf -n "${ARGO_NAMESPACE}" -o name 2>/dev/null | grep '^workflow.argoproj.io/vehicle-mapping-' || true
)"
if [[ -n "${mapping_workflows}" ]]; then
  while IFS= read -r workflow_ref; do
    [[ -z "${workflow_ref}" ]] && continue
    kubectl delete -n "${ARGO_NAMESPACE}" "${workflow_ref}" --ignore-not-found
  done <<< "${mapping_workflows}"
fi
kubectl delete -f "${K8S_DIR}/perception-cloud-service.yaml" --ignore-not-found
kubectl delete -f "${K8S_DIR}/perception-cloud-deployment.yaml" --ignore-not-found
kubectl delete -f "${K8S_DIR}/perception-cloud-configmap.yaml" --ignore-not-found
kubectl delete configmap vehicle-inventory -n "${ARGO_NAMESPACE}" --ignore-not-found

if [[ "${REMOVE_CLUSTER_INFRA}" == "true" ]]; then
  uninstall_vehicle_agent() {
    local host="$1"
    ssh "${SSH_OPTS[@]}" "${host}" "bash -lc '
      if [[ -x /usr/local/bin/k3s-agent-uninstall.sh ]]; then
        printf \"%s\n\" \"nvidia\" | sudo -S -p \"\" /usr/local/bin/k3s-agent-uninstall.sh
      elif [[ -x /usr/local/bin/k3s-uninstall.sh ]]; then
        printf \"%s\n\" \"nvidia\" | sudo -S -p \"\" /usr/local/bin/k3s-uninstall.sh
      fi
    '"
  }

  echo "Removing vehicle agents from cluster..."
  uninstall_vehicle_agent "${VEHICLE1_HOST}" || true
  uninstall_vehicle_agent "${VEHICLE2_HOST}" || true
  kubectl delete node vehicle1 --ignore-not-found
  kubectl delete node vehicle2 --ignore-not-found

  echo "Removing worker labels..."
  for node in ${GENERAL_WORKERS}; do
    kubectl label node "${node}" perception.role- >/dev/null 2>&1 || true
  done

  if [[ "${REMOVE_ARGO}" == "true" ]]; then
    echo "Removing Argo installation..."
    curl -fsSL "${ARGO_INSTALL_MANIFEST_URL}" | kubectl delete -f - >/dev/null 2>&1 || true
  fi

  kubectl delete namespace "${PERCEPTION_NAMESPACE}" --ignore-not-found >/dev/null 2>&1 || true
  kubectl delete namespace "${ARGO_NAMESPACE}" --ignore-not-found >/dev/null 2>&1 || true
else
  echo "Preserving vehicle k3s agents, nodes, Argo, and worker labels."
fi

echo "Cleanup finished."

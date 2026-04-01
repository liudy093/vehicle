#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/vehicle.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VEHICLE_ID="${VEHICLE_ID:-}"
VEHICLE_HOST="${VEHICLE_HOST:-}"
K3S_SERVER_URL="${K3S_SERVER_URL:-https://172.16.2.80:6443}"
K3S_SERVER_HOST="${K3S_SERVER_HOST:-root@172.16.2.80}"
K3S_TOKEN="${K3S_TOKEN:-}"
K3S_TOKEN_FILE="${K3S_TOKEN_FILE:-/var/lib/rancher/k3s/server/node-token}"
INSTALL_K3S_VERSION="${INSTALL_K3S_VERSION:-v1.34.5+k3s1}"
VEHICLE_SUDO_PASSWORD="${VEHICLE_SUDO_PASSWORD:-nvidia}"
SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null)
SSH_IDENTITY_FILE="${SSH_IDENTITY_FILE:-}"

if [[ -n "${SSH_IDENTITY_FILE}" && -f "${SSH_IDENTITY_FILE}" ]]; then
  SSH_OPTS+=(-i "${SSH_IDENTITY_FILE}")
fi

usage() {
  cat <<'EOF'
Usage:
  VEHICLE_ID=vehicle1 VEHICLE_HOST=root@192.168.3.2 ./deploy/shared/k8s/join_vehicle_k3s_agent.sh

Environment variables:
  VEHICLE_ID        Required. Node name / vehicle identifier.
  VEHICLE_HOST      Required. SSH target for the vehicle.
  K3S_SERVER_URL    Default: https://172.16.2.80:6443
  K3S_SERVER_HOST   Default: root@172.16.2.80
  K3S_TOKEN         Optional. If unset, fetched from the server host.
  K3S_TOKEN_FILE    Default: /var/lib/rancher/k3s/server/node-token
  INSTALL_K3S_VERSION Default: ${INSTALL_K3S_VERSION}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ -z "${VEHICLE_ID}" || -z "${VEHICLE_HOST}" ]]; then
  usage >&2
  exit 1
fi

for cmd in ssh kubectl; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "${cmd} is required" >&2
    exit 1
  fi
done

if [[ -z "${K3S_TOKEN}" ]]; then
  if [[ -r "${K3S_TOKEN_FILE}" ]]; then
    K3S_TOKEN="$(cat "${K3S_TOKEN_FILE}")"
  else
    K3S_TOKEN="$(ssh "${SSH_OPTS[@]}" "${K3S_SERVER_HOST}" "cat '${K3S_TOKEN_FILE}'")"
  fi
fi

NODE_LABELS=(
  "vehicle.role=vehicle"
  "vehicle.id=${VEHICLE_ID}"
)
NODE_TAINT="vehicle.role=vehicle:NoSchedule"
INSTALL_EXEC="agent --node-name ${VEHICLE_ID} --node-label ${NODE_LABELS[0]} --node-label ${NODE_LABELS[1]} --node-taint ${NODE_TAINT}"
REMOTE_USER="${VEHICLE_HOST%%@*}"

if [[ "${REMOTE_USER}" == "${VEHICLE_HOST}" ]]; then
  REMOTE_USER="${USER:-}"
fi

ssh "${SSH_OPTS[@]}" "${VEHICLE_HOST}" "bash -s" <<EOF
set -euo pipefail
if [[ '${REMOTE_USER}' == 'root' ]]; then
  curl -sfL https://get.k3s.io | \
    K3S_URL='${K3S_SERVER_URL}' \
    K3S_TOKEN='${K3S_TOKEN}' \
    INSTALL_K3S_VERSION='${INSTALL_K3S_VERSION}' \
    INSTALL_K3S_EXEC='${INSTALL_EXEC}' \
    sh -
  systemctl enable --now k3s-agent
else
  printf '%s\n' '${VEHICLE_SUDO_PASSWORD}' | sudo -S -p '' env \
    K3S_URL='${K3S_SERVER_URL}' \
    K3S_TOKEN='${K3S_TOKEN}' \
    INSTALL_K3S_VERSION='${INSTALL_K3S_VERSION}' \
    INSTALL_K3S_EXEC='${INSTALL_EXEC}' \
    sh -c "curl -sfL https://get.k3s.io | sh -"
  printf '%s\n' '${VEHICLE_SUDO_PASSWORD}' | sudo -S -p '' systemctl enable --now k3s-agent
fi
EOF

for _ in $(seq 1 60); do
  if kubectl get node "${VEHICLE_ID}" >/dev/null 2>&1; then
    break
  fi
  sleep 5
done

kubectl wait --for=condition=Ready "node/${VEHICLE_ID}" --timeout=5m
kubectl label node "${VEHICLE_ID}" "${NODE_LABELS[0]}" "${NODE_LABELS[1]}" --overwrite
kubectl taint node "${VEHICLE_ID}" "${NODE_TAINT}" --overwrite

echo "Registered ${VEHICLE_ID} as a tainted k3s agent."

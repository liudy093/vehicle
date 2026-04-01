#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/vehicle.k3s.yaml

WORKER_IDS="${WORKER_IDS:-k3s-worker-1 k3s-worker-2 k3s-worker-3}"

if ! command -v kubectl >/dev/null 2>&1; then
  echo "kubectl is required" >&2
  exit 1
fi

for node in ${WORKER_IDS}; do
  kubectl label node "${node}" vehicle.role=general --overwrite
done

cat <<'EOF'
Labeled general worker nodes:
- k3s-worker-1 (172.16.2.81)
- k3s-worker-2 (172.16.2.82)
- k3s-worker-3 (172.16.2.83)
EOF

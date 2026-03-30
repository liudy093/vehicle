#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/perception.k3s.yaml

ARGO_NAMESPACE="${ARGO_NAMESPACE:-argo}"
PERCEPTION_NAMESPACE="${PERCEPTION_NAMESPACE:-perception}"
ARGO_WORKFLOWS_VERSION="${ARGO_WORKFLOWS_VERSION:-v3.7.12}"
ARGO_INSTALL_MANIFEST_URL="${ARGO_INSTALL_MANIFEST_URL:-https://github.com/argoproj/argo-workflows/releases/download/${ARGO_WORKFLOWS_VERSION}/install.yaml}"
APPLY_ARGO_INSTALL_MANIFEST="${APPLY_ARGO_INSTALL_MANIFEST:-true}"
ARGO_EXECUTOR_BYPASS_KUBE_SERVICE="${ARGO_EXECUTOR_BYPASS_KUBE_SERVICE:-true}"
ARGO_EXECUTOR_KUBE_API_SERVER_HOST="${ARGO_EXECUTOR_KUBE_API_SERVER_HOST:-172.16.2.80}"
ARGO_EXECUTOR_KUBE_API_SERVER_PORT="${ARGO_EXECUTOR_KUBE_API_SERVER_PORT:-6443}"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "$1 is required" >&2
    exit 1
  fi
}

require_cmd kubectl

kubectl create namespace "${ARGO_NAMESPACE}" --dry-run=client -o yaml | kubectl apply -f -
kubectl create namespace "${PERCEPTION_NAMESPACE}" --dry-run=client -o yaml | kubectl apply -f -

if [[ "${APPLY_ARGO_INSTALL_MANIFEST}" == "true" ]]; then
  require_cmd curl
  curl -fsSL "${ARGO_INSTALL_MANIFEST_URL}" | kubectl apply -n "${ARGO_NAMESPACE}" -f -
else
  echo "Skipping Argo install manifest apply; reconciling controller config only."
fi

executor_env_block=""
if [[ "${ARGO_EXECUTOR_BYPASS_KUBE_SERVICE}" == "true" ]]; then
  executor_env_block=$(cat <<EOF
    # Vehicle nodes can miss kube-proxy Service NAT rules; point wait/init traffic at the control-plane endpoint directly.
    executor:
      env:
        - name: KUBERNETES_SERVICE_HOST
          value: "${ARGO_EXECUTOR_KUBE_API_SERVER_HOST}"
        - name: KUBERNETES_SERVICE_PORT
          value: "${ARGO_EXECUTOR_KUBE_API_SERVER_PORT}"
EOF
)
fi

# Argo v3.4+ only supports the emissary executor. Do not pin legacy executors.
kubectl apply -f - <<EOF
apiVersion: v1
kind: ConfigMap
metadata:
  name: workflow-controller-configmap
  namespace: ${ARGO_NAMESPACE}
data:
  config: |
    # Argo ${ARGO_WORKFLOWS_VERSION} uses the supported emissary executor path by default.
${executor_env_block}
EOF

kubectl create rolebinding argo-perception-admin \
  -n "${PERCEPTION_NAMESPACE}" \
  --clusterrole=admin \
  --serviceaccount="${ARGO_NAMESPACE}:argo" \
  --dry-run=client \
  -o yaml | kubectl apply -f -

kubectl create rolebinding argo-workflowtaskresults-writer \
  -n "${ARGO_NAMESPACE}" \
  --clusterrole=argo-aggregate-to-admin \
  --serviceaccount="${ARGO_NAMESPACE}:argo" \
  --dry-run=client \
  -o yaml | kubectl apply -f -

kubectl rollout restart deployment/workflow-controller -n "${ARGO_NAMESPACE}"

kubectl rollout status deployment/workflow-controller -n "${ARGO_NAMESPACE}" --timeout=5m
kubectl rollout status deployment/argo-server -n "${ARGO_NAMESPACE}" --timeout=5m

echo "Argo Workflows installed in namespace ${ARGO_NAMESPACE}."

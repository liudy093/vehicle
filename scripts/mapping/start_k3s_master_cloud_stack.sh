#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/mapping/start_k3s_master_cloud_stack.sh

Environment variables:
  CLOUD_HOST              Default: root@172.16.2.80
  CLOUD_HOST_IP           Default: 172.16.2.80
  IMAGE_NAME              Default: vehicle-cloud:amd64
  CONTAINER_NAME          Default: vehicle-cloud
  SERVER_PORT             Default: 15000
  VIEWER_PORT             Default: 18080
  WEB_VIEWER_TOPIC        Default: /vehicle1/cloud_sent/forward
  WEB_VIEWER_TOPICS       Default: vehicle1/fused=/vehicle1/cloud_sent/fused,vehicle1/global_cloud=/vehicle1/global_cloud,vehicle2/fused=/vehicle2/cloud_sent/fused,vehicle2/global_cloud=/vehicle2/global_cloud,global/global_map=/global_map,global/global_map_optimized=/global_map_optimized
  WEB_VIEWER_MAX_POINTS   Default: 20000
  FUSED_CLOUD_TOPIC_NAME  Default: fused
  FUSED_LIDAR_IDS         Default: forward,left,right
  FUSED_LIDAR_EXTRINSICS  Default: forward:0.0,0.0,0.0,0;left:-0.8,-0.6,0.0,90;right:0.8,-0.6,0.0,-90
  GLOBAL_MAP_LEAF_SIZE    Default: 0.15
  GLOBAL_FRAME_LEAF_SIZE  Default: 0.1
  IMAGE_TAR               Default: /tmp/vehicle-cloud.amd64.tar
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

CLOUD_HOST="${CLOUD_HOST:-root@172.16.2.80}"
CLOUD_HOST_IP="${CLOUD_HOST_IP:-172.16.2.80}"
IMAGE_NAME="${IMAGE_NAME:-vehicle-cloud:amd64}"
CONTAINER_NAME="${CONTAINER_NAME:-vehicle-cloud}"
SERVER_PORT="${SERVER_PORT:-15000}"
VIEWER_PORT="${VIEWER_PORT:-18080}"
WEB_VIEWER_TOPIC="${WEB_VIEWER_TOPIC:-/vehicle1/cloud_sent/forward}"
WEB_VIEWER_TOPICS="${WEB_VIEWER_TOPICS:-vehicle1/fused=/vehicle1/cloud_sent/fused,vehicle1/global_cloud=/vehicle1/global_cloud,vehicle2/fused=/vehicle2/cloud_sent/fused,vehicle2/global_cloud=/vehicle2/global_cloud,global/global_map=/global_map,global/global_map_optimized=/global_map_optimized}"
WEB_VIEWER_MAX_POINTS="${WEB_VIEWER_MAX_POINTS:-20000}"
FUSED_CLOUD_TOPIC_NAME="${FUSED_CLOUD_TOPIC_NAME:-fused}"
FUSED_LIDAR_IDS="${FUSED_LIDAR_IDS:-forward,left,right}"
FUSED_LIDAR_EXTRINSICS="${FUSED_LIDAR_EXTRINSICS:-forward:0.0,0.0,0.0,0;left:-0.8,-0.6,0.0,90;right:0.8,-0.6,0.0,-90}"
GLOBAL_MAP_LEAF_SIZE="${GLOBAL_MAP_LEAF_SIZE:-0.15}"
GLOBAL_FRAME_LEAF_SIZE="${GLOBAL_FRAME_LEAF_SIZE:-0.1}"
IMAGE_TAR="${IMAGE_TAR:-/tmp/vehicle-cloud.amd64.tar}"
REMOTE_IMAGE_TAR="/tmp/$(basename "${IMAGE_TAR}")"
NERDCTL="nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io"

for cmd in ssh scp; do
  if ! command -v "${cmd}" >/dev/null 2>&1; then
    echo "${cmd} is required" >&2
    exit 1
  fi
done

if [[ ! -f "${IMAGE_TAR}" ]]; then
  echo "Image tar not found: ${IMAGE_TAR}" >&2
  exit 1
fi

scp "${IMAGE_TAR}" "${CLOUD_HOST}:${REMOTE_IMAGE_TAR}"

ssh "${CLOUD_HOST}" "bash -lc '
set -euo pipefail
${NERDCTL} load -i \"${REMOTE_IMAGE_TAR}\"
${NERDCTL} rm -f ${CONTAINER_NAME} >/dev/null 2>&1 || true
${NERDCTL} run -d \
  --name ${CONTAINER_NAME} \
  --net host \
  -e SERVER_HOST=0.0.0.0 \
  -e SERVER_PORT=${SERVER_PORT} \
  -e WEB_VIEWER_TOPIC=\"${WEB_VIEWER_TOPIC}\" \
  -e WEB_VIEWER_TOPICS=\"${WEB_VIEWER_TOPICS}\" \
  -e WEB_VIEWER_MAX_POINTS=\"${WEB_VIEWER_MAX_POINTS}\" \
  -e FUSED_CLOUD_TOPIC_NAME=\"${FUSED_CLOUD_TOPIC_NAME}\" \
  -e FUSED_LIDAR_IDS=\"${FUSED_LIDAR_IDS}\" \
  -e FUSED_LIDAR_EXTRINSICS=\"${FUSED_LIDAR_EXTRINSICS}\" \
  -e GLOBAL_MAP_LEAF_SIZE=\"${GLOBAL_MAP_LEAF_SIZE}\" \
  -e GLOBAL_FRAME_LEAF_SIZE=\"${GLOBAL_FRAME_LEAF_SIZE}\" \
  ${IMAGE_NAME}
'"

cat <<EOF
Cloud container started on k3s-master.
Cloud host:             ${CLOUD_HOST}
Cloud receive endpoint: http://${CLOUD_HOST_IP}:${SERVER_PORT}
Browser viewer:         http://${CLOUD_HOST_IP}:${VIEWER_PORT}
Viewer topic:           ${WEB_VIEWER_TOPIC}
Viewer topics:          ${WEB_VIEWER_TOPICS}
Viewer max points:      ${WEB_VIEWER_MAX_POINTS}
Fused topic:            /cloud_sent/${FUSED_CLOUD_TOPIC_NAME}
Fused lidar ids:        ${FUSED_LIDAR_IDS}
Fused extrinsics:       ${FUSED_LIDAR_EXTRINSICS}
Global map leaf size:   ${GLOBAL_MAP_LEAF_SIZE}
Global frame leaf size: ${GLOBAL_FRAME_LEAF_SIZE}
EOF

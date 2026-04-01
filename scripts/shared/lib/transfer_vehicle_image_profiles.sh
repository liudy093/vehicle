#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/vehicle.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SHARED_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
source "${SCRIPT_DIR}/common.sh"
source "${SHARED_DIR}/vehicle_inventory.sh"
IMAGE_PROFILE="${IMAGE_PROFILE:-all}"
WORKER_HOSTS="${WORKER_HOSTS:-root@172.16.2.81 root@172.16.2.82 root@172.16.2.83}"
VEHICLE_HOSTS="${VEHICLE_HOSTS:-$(vehicle_inventory_host "vehicle1" "nvidia") $(vehicle_inventory_host "vehicle2" "nvidia")}"
VEHICLE_SUDO_PASSWORD="${VEHICLE_SUDO_PASSWORD:-nvidia}"
ARGO_WORKFLOWS_VERSION="${ARGO_WORKFLOWS_VERSION:-v3.7.12}"
VEHICLE_CLOUD_IMAGE_TAR="${VEHICLE_CLOUD_IMAGE_TAR:-/tmp/vehicle-cloud.amd64.tar}"
SSH_RUNNER_IMAGE_TAR="${SSH_RUNNER_IMAGE_TAR:-/tmp/vehicle-ssh-runner.amd64.tar}"
VEHICLE_RUNNER_IMAGE_TAR="${VEHICLE_RUNNER_IMAGE_TAR:-/tmp/vehicle-autodrive-runner.arm64.tar}"
VEHICLE_MAPPING_RUNNER_IMAGE_TAR="${VEHICLE_MAPPING_RUNNER_IMAGE_TAR:-/tmp/vehicle-mapping-runner.arm64.tar}"
VEHICLE_ARGOEXEC_IMAGE_TAR="${VEHICLE_ARGOEXEC_IMAGE_TAR:-/tmp/argoexec-${ARGO_WORKFLOWS_VERSION}.arm64.tar}"
VEHICLE_PAUSE_IMAGE_TAR="${VEHICLE_PAUSE_IMAGE_TAR:-/tmp/rancher-mirrored-pause-3.6.arm64.tar}"
ARGOEXEC_IMAGE_TAR="${ARGOEXEC_IMAGE_TAR:-/tmp/argoexec-${ARGO_WORKFLOWS_VERSION}.amd64.tar}"
REMOTE_IMAGE_CACHE_DIR="${REMOTE_IMAGE_CACHE_DIR:-/var/lib/vehicle/image-cache}"
SSH_OPTS=()
SSH_IDENTITY_FILE="${SSH_IDENTITY_FILE:-}"
IMAGE_SCRIPT_NAME="${IMAGE_SCRIPT_NAME:-./scripts/shared/transfer_vehicle_image_profiles.sh}"

usage() {
  cat <<EOF
Usage:
  ${IMAGE_SCRIPT_NAME}

Environment variables:
  IMAGE_PROFILE             Default: ${IMAGE_PROFILE} (all, autodrive, or mapping)
  WORKER_HOSTS               Default: ${WORKER_HOSTS}
  VEHICLE_HOSTS              Default: ${VEHICLE_HOSTS}
  VEHICLE_SUDO_PASSWORD      Default: ${VEHICLE_SUDO_PASSWORD}
  ARGO_WORKFLOWS_VERSION     Default: ${ARGO_WORKFLOWS_VERSION}
  VEHICLE_CLOUD_IMAGE_TAR    Default: ${VEHICLE_CLOUD_IMAGE_TAR}
  SSH_RUNNER_IMAGE_TAR       Default: ${SSH_RUNNER_IMAGE_TAR}
  VEHICLE_RUNNER_IMAGE_TAR   Default: ${VEHICLE_RUNNER_IMAGE_TAR}
  VEHICLE_MAPPING_RUNNER_IMAGE_TAR Default: ${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}
  VEHICLE_ARGOEXEC_IMAGE_TAR Default: ${VEHICLE_ARGOEXEC_IMAGE_TAR}
  VEHICLE_PAUSE_IMAGE_TAR    Default: ${VEHICLE_PAUSE_IMAGE_TAR}
  ARGOEXEC_IMAGE_TAR         Default: ${ARGOEXEC_IMAGE_TAR}
  REMOTE_IMAGE_CACHE_DIR     Default: ${REMOTE_IMAGE_CACHE_DIR}
  SSH_IDENTITY_FILE          Default: ${SSH_IDENTITY_FILE:-<unset>}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

case "${IMAGE_PROFILE}" in
  all|autodrive|mapping)
    ;;
  *)
    echo "IMAGE_PROFILE must be one of: all, autodrive, mapping" >&2
    usage >&2
    exit 1
    ;;
esac

needs_worker_cloud=false
needs_worker_ssh_runner=false
needs_worker_argoexec=false
needs_vehicle_runner=false
needs_vehicle_mapping_runner=false
needs_vehicle_argoexec=false
needs_vehicle_pause=false

case "${IMAGE_PROFILE}" in
  all)
    needs_worker_cloud=true
    needs_worker_ssh_runner=true
    needs_worker_argoexec=true
    needs_vehicle_runner=true
    needs_vehicle_mapping_runner=true
    needs_vehicle_argoexec=true
    needs_vehicle_pause=true
    ;;
  autodrive)
    needs_vehicle_runner=true
    needs_vehicle_argoexec=true
    needs_vehicle_pause=true
    ;;
  mapping)
    needs_worker_cloud=true
    needs_worker_ssh_runner=true
    needs_worker_argoexec=true
    needs_vehicle_mapping_runner=true
    needs_vehicle_argoexec=true
    needs_vehicle_pause=true
    ;;
esac

while IFS= read -r opt; do
  SSH_OPTS+=("${opt}")
done < <(build_ssh_opts "${SSH_IDENTITY_FILE}")

for cmd in scp ssh python3; do
  require_cmd "${cmd}"
done

required_image_tars=()
if [[ "${needs_worker_cloud}" == "true" ]]; then
  required_image_tars+=("${VEHICLE_CLOUD_IMAGE_TAR}")
fi
if [[ "${needs_worker_ssh_runner}" == "true" ]]; then
  required_image_tars+=("${SSH_RUNNER_IMAGE_TAR}")
fi
if [[ "${needs_vehicle_runner}" == "true" ]]; then
  required_image_tars+=("${VEHICLE_RUNNER_IMAGE_TAR}")
fi
if [[ "${needs_vehicle_mapping_runner}" == "true" ]]; then
  required_image_tars+=("${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}")
fi
if [[ "${needs_vehicle_argoexec}" == "true" ]]; then
  required_image_tars+=("${VEHICLE_ARGOEXEC_IMAGE_TAR}")
fi
if [[ "${needs_vehicle_pause}" == "true" ]]; then
  required_image_tars+=("${VEHICLE_PAUSE_IMAGE_TAR}")
fi

for image_tar in "${required_image_tars[@]}"; do
  if [[ ! -f "${image_tar}" ]]; then
    echo "Image tar not found: ${image_tar}" >&2
    exit 1
  fi
done

has_local_argoexec_tar=false
if [[ "${needs_worker_argoexec}" == "true" && -f "${ARGOEXEC_IMAGE_TAR}" ]]; then
  has_local_argoexec_tar=true
fi

if command -v sha256sum >/dev/null 2>&1; then
  hash_cmd=(sha256sum)
elif command -v shasum >/dev/null 2>&1; then
  hash_cmd=(shasum -a 256)
else
  echo "sha256sum or shasum is required" >&2
  exit 1
fi

if [[ "${needs_worker_cloud}" == "true" ]]; then
  cloud_hash="$("${hash_cmd[@]}" "${VEHICLE_CLOUD_IMAGE_TAR}" | awk '{print $1}')"
fi
if [[ "${needs_worker_ssh_runner}" == "true" ]]; then
  runner_hash="$("${hash_cmd[@]}" "${SSH_RUNNER_IMAGE_TAR}" | awk '{print $1}')"
fi
if [[ "${needs_vehicle_runner}" == "true" ]]; then
  vehicle_runner_hash="$("${hash_cmd[@]}" "${VEHICLE_RUNNER_IMAGE_TAR}" | awk '{print $1}')"
fi
if [[ "${needs_vehicle_mapping_runner}" == "true" ]]; then
  vehicle_mapping_runner_hash="$("${hash_cmd[@]}" "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" | awk '{print $1}')"
fi
if [[ "${needs_vehicle_argoexec}" == "true" ]]; then
  vehicle_argoexec_hash="$("${hash_cmd[@]}" "${VEHICLE_ARGOEXEC_IMAGE_TAR}" | awk '{print $1}')"
fi
if [[ "${needs_vehicle_pause}" == "true" ]]; then
  vehicle_pause_hash="$("${hash_cmd[@]}" "${VEHICLE_PAUSE_IMAGE_TAR}" | awk '{print $1}')"
fi

repo_tag_from_tar() {
  local image_tar="$1"

  python3 - "${image_tar}" <<'PY'
import json
import sys
import tarfile

image_tar = sys.argv[1]
with tarfile.open(image_tar, "r") as archive:
    manifest = json.load(archive.extractfile("manifest.json"))

for entry in manifest:
    tags = entry.get("RepoTags") or []
    if tags:
        print(tags[0])
        sys.exit(0)

raise SystemExit(f"no RepoTags found in {image_tar}")
PY
}

normalize_image_ref() {
  local image_ref="$1"
  local name_part="${image_ref%%:*}"

  if [[ "${name_part}" == *"/"* ]]; then
    local first_segment="${name_part%%/*}"
    if [[ "${first_segment}" == *"."* || "${first_segment}" == *":"* || "${first_segment}" == "localhost" ]]; then
      printf '%s\n' "${image_ref}"
      return 0
    fi
  fi

  if [[ "${name_part}" == *"/"* ]]; then
    printf 'docker.io/%s\n' "${image_ref}"
  else
    printf 'docker.io/library/%s\n' "${image_ref}"
  fi
}

if [[ "${needs_worker_cloud}" == "true" ]]; then
  VEHICLE_CLOUD_IMAGE_REF="${VEHICLE_CLOUD_IMAGE_REF:-$(normalize_image_ref "$(repo_tag_from_tar "${VEHICLE_CLOUD_IMAGE_TAR}")")}"
fi
if [[ "${needs_worker_ssh_runner}" == "true" ]]; then
  SSH_RUNNER_IMAGE_REF="${SSH_RUNNER_IMAGE_REF:-$(normalize_image_ref "$(repo_tag_from_tar "${SSH_RUNNER_IMAGE_TAR}")")}"
fi
if [[ "${needs_vehicle_runner}" == "true" ]]; then
  VEHICLE_RUNNER_IMAGE_REF="${VEHICLE_RUNNER_IMAGE_REF:-$(normalize_image_ref "$(repo_tag_from_tar "${VEHICLE_RUNNER_IMAGE_TAR}")")}"
fi
if [[ "${needs_vehicle_mapping_runner}" == "true" ]]; then
  VEHICLE_MAPPING_RUNNER_IMAGE_REF="${VEHICLE_MAPPING_RUNNER_IMAGE_REF:-$(normalize_image_ref "$(repo_tag_from_tar "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}")")}"
fi
if [[ "${needs_vehicle_pause}" == "true" ]]; then
  VEHICLE_PAUSE_IMAGE_REF="${VEHICLE_PAUSE_IMAGE_REF:-$(normalize_image_ref "$(repo_tag_from_tar "${VEHICLE_PAUSE_IMAGE_TAR}")")}"
fi

image_exists_in_runtime() {
  local worker="$1"
  local image_ref="$2"
  local use_sudo="${3:-false}"

  if [[ "${use_sudo}" == "true" ]]; then
    ssh "${SSH_OPTS[@]}" "${worker}" "bash -s" -- "${image_ref}" "${VEHICLE_SUDO_PASSWORD}" <<'EOF'
set -euo pipefail
image_ref="$1"
sudo_password="$2"
sudo_run() { printf '%s\n' "${sudo_password}" | sudo -S -p '' "$@"; }
if command -v nerdctl >/dev/null 2>&1; then
  sudo_run nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io image inspect "${image_ref}" >/dev/null 2>&1
else
  sudo_run k3s ctr --namespace k8s.io images ls | awk '{print $1}' | grep -Fx "${image_ref}" >/dev/null 2>&1
fi
EOF
  else
    ssh "${SSH_OPTS[@]}" "${worker}" "bash -s" -- "${image_ref}" <<'EOF'
set -euo pipefail
image_ref="$1"
if command -v nerdctl >/dev/null 2>&1; then
  nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io image inspect "${image_ref}" >/dev/null 2>&1
else
  k3s ctr --namespace k8s.io images ls | awk '{print $1}' | grep -Fx "${image_ref}" >/dev/null 2>&1
fi
EOF
  fi
}

verify_imported_image() {
  local worker="$1"
  local image_ref="$2"
  local use_sudo="${3:-false}"

  if ! image_exists_in_runtime "${worker}" "${image_ref}" "${use_sudo}"; then
    echo "Imported image not visible in runtime on ${worker}: ${image_ref}" >&2
    exit 1
  fi
}

sync_image_if_needed() {
  local worker="$1"
  local image_tar="$2"
  local expected_hash="$3"
  local image_name="$4"
  local image_ref="$5"
  local remote_tar="/tmp/$(basename "${image_tar}")"
  local remote_hash_file="${REMOTE_IMAGE_CACHE_DIR}/${image_name}.sha256"
  local remote_hash

  remote_hash="$(
    ssh "${SSH_OPTS[@]}" "${worker}" "bash -lc '
      set -euo pipefail
      if [[ -f \"${remote_hash_file}\" ]]; then
        cat \"${remote_hash_file}\"
      fi
    '"
  )"

  if [[ "${remote_hash}" == "${expected_hash}" ]] && image_exists_in_runtime "${worker}" "${image_ref}"; then
    echo "Skipping transfer/import for ${image_name} on ${worker}; hash unchanged."
    return 0
  fi

  scp "${SSH_OPTS[@]}" "${image_tar}" "${worker}:${remote_tar}"
  ssh "${SSH_OPTS[@]}" "${worker}" "bash -lc '
    set -euo pipefail
    mkdir -p \"${REMOTE_IMAGE_CACHE_DIR}\"
    if command -v nerdctl >/dev/null 2>&1; then
      nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io load -i \"${remote_tar}\"
    else
      k3s ctr --namespace k8s.io images import \"${remote_tar}\"
    fi
  '"
  verify_imported_image "${worker}" "${image_ref}"
  ssh "${SSH_OPTS[@]}" "${worker}" "bash -lc '
    set -euo pipefail
    mkdir -p \"${REMOTE_IMAGE_CACHE_DIR}\"
    printf \"%s\" \"${expected_hash}\" > \"${remote_hash_file}\"
  '"
}

ensure_argoexec_image() {
  local worker="$1"

  ssh "${SSH_OPTS[@]}" "${worker}" "bash -lc '
    set -euo pipefail
    if command -v nerdctl >/dev/null 2>&1; then
      if nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io image inspect quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION} >/dev/null 2>&1 || \
         nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io image inspect docker.io/library/vehicle-argoexec:${ARGO_WORKFLOWS_VERSION} >/dev/null 2>&1; then
        echo \"reusing existing argoexec image on ${worker}\"
      else
        exit 10
      fi
    else
      if k3s ctr --namespace k8s.io images ls | grep -F \"quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION}\" >/dev/null 2>&1 || \
         k3s ctr --namespace k8s.io images ls | grep -F \"docker.io/library/vehicle-argoexec:${ARGO_WORKFLOWS_VERSION}\" >/dev/null 2>&1; then
        echo \"reusing existing argoexec image on ${worker}\"
      else
        exit 10
      fi
    fi
  '"
}

print_worker_images() {
  local worker="$1"

  ssh "${SSH_OPTS[@]}" "${worker}" "bash -lc '
    set -euo pipefail
    if command -v nerdctl >/dev/null 2>&1; then
      nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io images | egrep \"vehicle-cloud|vehicle-ssh-runner|argoexec\"
    else
      k3s ctr --namespace k8s.io images ls | egrep \"vehicle-cloud|vehicle-ssh-runner|argoexec\"
    fi
  '"
}

sync_vehicle_image_if_needed() {
  local worker="$1"
  local image_tar="$2"
  local expected_hash="$3"
  local image_name="$4"
  local image_ref="$5"
  local remote_tar="/tmp/$(basename "${image_tar}")"
  local remote_hash_file="${REMOTE_IMAGE_CACHE_DIR}/${image_name}.sha256"
  local remote_hash

  remote_hash="$(
    ssh "${SSH_OPTS[@]}" "${worker}" "bash -s" <<EOF
set -euo pipefail
sudo_run() { printf '%s\n' '${VEHICLE_SUDO_PASSWORD}' | sudo -S -p '' "\$@"; }
if sudo_run test -f "${remote_hash_file}"; then
  sudo_run cat "${remote_hash_file}"
fi
EOF
  )"

  if [[ "${remote_hash}" == "${expected_hash}" ]] && image_exists_in_runtime "${worker}" "${image_ref}" "true"; then
    echo "Skipping transfer/import for ${image_name} on ${worker}; hash unchanged."
    return 0
  fi

  scp "${SSH_OPTS[@]}" "${image_tar}" "${worker}:${remote_tar}"
  ssh "${SSH_OPTS[@]}" "${worker}" "bash -s" <<EOF
set -euo pipefail
sudo_run() { printf '%s\n' '${VEHICLE_SUDO_PASSWORD}' | sudo -S -p '' "\$@"; }
sudo_run mkdir -p "${REMOTE_IMAGE_CACHE_DIR}"
if command -v nerdctl >/dev/null 2>&1; then
  sudo_run nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io load -i "${remote_tar}"
else
  sudo_run k3s ctr --namespace k8s.io images import "${remote_tar}"
fi
EOF
  verify_imported_image "${worker}" "${image_ref}" "true"
  ssh "${SSH_OPTS[@]}" "${worker}" "bash -s" <<EOF
set -euo pipefail
sudo_run() { printf '%s\n' '${VEHICLE_SUDO_PASSWORD}' | sudo -S -p '' "\$@"; }
printf '%s' '${expected_hash}' > /tmp/${image_name}.sha256
sudo_run mkdir -p "${REMOTE_IMAGE_CACHE_DIR}"
sudo_run install -m 0644 /tmp/${image_name}.sha256 "${remote_hash_file}"
rm -f /tmp/${image_name}.sha256
EOF
}

print_vehicle_images() {
  local worker="$1"

  ssh "${SSH_OPTS[@]}" "${worker}" "bash -s" <<EOF
set -euo pipefail
sudo_run() { printf '%s\n' '${VEHICLE_SUDO_PASSWORD}' | sudo -S -p '' "\$@"; }
if command -v nerdctl >/dev/null 2>&1; then
  sudo_run nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io images | egrep "vehicle-cloud|vehicle-ssh-runner|vehicle-autodrive-runner|vehicle-mapping-runner|argoexec|mirrored-pause"
else
  sudo_run k3s ctr --namespace k8s.io images ls | egrep "vehicle-cloud|vehicle-ssh-runner|vehicle-autodrive-runner|vehicle-mapping-runner|argoexec|mirrored-pause"
fi
EOF
}

if [[ "${needs_worker_cloud}" == "true" || "${needs_worker_ssh_runner}" == "true" || "${needs_worker_argoexec}" == "true" ]]; then
  for worker in ${WORKER_HOSTS}; do
    remote_argoexec_tar="/tmp/$(basename "${ARGOEXEC_IMAGE_TAR}")"

    if [[ "${needs_worker_cloud}" == "true" ]]; then
      sync_image_if_needed "${worker}" "${VEHICLE_CLOUD_IMAGE_TAR}" "${cloud_hash}" "vehicle-cloud" "${VEHICLE_CLOUD_IMAGE_REF}"
    fi

    if [[ "${needs_worker_ssh_runner}" == "true" ]]; then
      sync_image_if_needed "${worker}" "${SSH_RUNNER_IMAGE_TAR}" "${runner_hash}" "vehicle-ssh-runner" "${SSH_RUNNER_IMAGE_REF}"
    fi

    if [[ "${needs_worker_argoexec}" == "true" ]]; then
      ensure_argoexec_image "${worker}" || {
        if [[ "${has_local_argoexec_tar}" != "true" ]]; then
          echo "ARGOEXEC missing on worker and fallback tar not found: ${worker} -> ${ARGOEXEC_IMAGE_TAR}" >&2
          exit 1
        fi

        scp "${SSH_OPTS[@]}" "${ARGOEXEC_IMAGE_TAR}" "${worker}:${remote_argoexec_tar}"
        ssh "${SSH_OPTS[@]}" "${worker}" "bash -lc '
    set -euo pipefail
    if command -v nerdctl >/dev/null 2>&1; then
      nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io load -i \"${remote_argoexec_tar}\"
      if ! nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io image inspect quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION} >/dev/null 2>&1 && \
         nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io image inspect docker.io/library/vehicle-argoexec:${ARGO_WORKFLOWS_VERSION} >/dev/null 2>&1; then
        nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io tag \
          docker.io/library/vehicle-argoexec:${ARGO_WORKFLOWS_VERSION} \
          quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION}
      fi
      nerdctl --address /run/k3s/containerd/containerd.sock --namespace k8s.io images | egrep \"vehicle-cloud|vehicle-ssh-runner|argoexec\"
    else
      k3s ctr --namespace k8s.io images import \"${remote_argoexec_tar}\"
      if ! k3s ctr --namespace k8s.io images ls | grep -F \"quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION}\" >/dev/null 2>&1 && \
         k3s ctr --namespace k8s.io images ls | grep -F \"docker.io/library/vehicle-argoexec:${ARGO_WORKFLOWS_VERSION}\" >/dev/null 2>&1; then
        k3s ctr --namespace k8s.io images tag \
          docker.io/library/vehicle-argoexec:${ARGO_WORKFLOWS_VERSION} \
          quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION}
      fi
    fi
    '"
        verify_imported_image "${worker}" "quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION}"
      }
    fi

    print_worker_images "${worker}"
  done
fi

if [[ "${needs_vehicle_runner}" == "true" || "${needs_vehicle_mapping_runner}" == "true" || "${needs_vehicle_argoexec}" == "true" || "${needs_vehicle_pause}" == "true" ]]; then
  for worker in ${VEHICLE_HOSTS}; do
    if [[ "${needs_vehicle_runner}" == "true" ]]; then
      sync_vehicle_image_if_needed "${worker}" "${VEHICLE_RUNNER_IMAGE_TAR}" "${vehicle_runner_hash}" "vehicle-autodrive-runner" "${VEHICLE_RUNNER_IMAGE_REF}"
    fi

    if [[ "${needs_vehicle_mapping_runner}" == "true" ]]; then
      sync_vehicle_image_if_needed "${worker}" "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" "${vehicle_mapping_runner_hash}" "vehicle-mapping-runner" "${VEHICLE_MAPPING_RUNNER_IMAGE_REF}"
    fi

    if [[ "${needs_vehicle_argoexec}" == "true" ]]; then
      sync_vehicle_image_if_needed "${worker}" "${VEHICLE_ARGOEXEC_IMAGE_TAR}" "${vehicle_argoexec_hash}" "argoproj-argoexec-arm64" "quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION}"
    fi

    if [[ "${needs_vehicle_pause}" == "true" ]]; then
      sync_vehicle_image_if_needed "${worker}" "${VEHICLE_PAUSE_IMAGE_TAR}" "${vehicle_pause_hash}" "rancher-mirrored-pause-3.6-arm64" "${VEHICLE_PAUSE_IMAGE_REF}"
    fi

    print_vehicle_images "${worker}"
  done
fi

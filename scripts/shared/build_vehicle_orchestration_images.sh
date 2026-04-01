#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/perception.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
SSH_RUNNER_DOCKERFILE="${ROOT_DIR}/docker/shared/ssh-runner/Dockerfile"
VEHICLE_RUNNER_DOCKERFILE="${VEHICLE_RUNNER_DOCKERFILE:-${ROOT_DIR}/docker/autodrive/vehicle-runner/Dockerfile}"
VEHICLE_MAPPING_RUNNER_DOCKERFILE="${ROOT_DIR}/docker/mapping/vehicle-mapping-runner/Dockerfile"
source "${SCRIPT_DIR}/lib/common.sh"

ARGO_WORKFLOWS_VERSION="${ARGO_WORKFLOWS_VERSION:-v3.7.12}"
IMAGE_PROFILE="${IMAGE_PROFILE:-all}"
SSH_RUNNER_IMAGE="${SSH_RUNNER_IMAGE:-vehicle-ssh-runner:latest}"
SSH_RUNNER_IMAGE_TAR="${SSH_RUNNER_IMAGE_TAR:-/tmp/vehicle-ssh-runner.amd64.tar}"
SSH_RUNNER_BUILD_STATE_FILE="${SSH_RUNNER_BUILD_STATE_FILE:-${SSH_RUNNER_IMAGE_TAR}.build-context.sha256}"
VEHICLE_RUNNER_IMAGE="${VEHICLE_RUNNER_IMAGE:-vehicle-autodrive-runner:latest}"
VEHICLE_RUNNER_IMAGE_TAR="${VEHICLE_RUNNER_IMAGE_TAR:-/tmp/vehicle-autodrive-runner.arm64.tar}"
VEHICLE_RUNNER_BUILD_STATE_FILE="${VEHICLE_RUNNER_BUILD_STATE_FILE:-${VEHICLE_RUNNER_IMAGE_TAR}.build-context.sha256}"
VEHICLE_MAPPING_RUNNER_IMAGE="${VEHICLE_MAPPING_RUNNER_IMAGE:-vehicle-mapping-runner:latest}"
VEHICLE_MAPPING_RUNNER_IMAGE_TAR="${VEHICLE_MAPPING_RUNNER_IMAGE_TAR:-/tmp/vehicle-mapping-runner.arm64.tar}"
VEHICLE_MAPPING_RUNNER_BUILD_STATE_FILE="${VEHICLE_MAPPING_RUNNER_BUILD_STATE_FILE:-${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}.build-context.sha256}"
VEHICLE_ARGOEXEC_IMAGE="${VEHICLE_ARGOEXEC_IMAGE:-quay.io/argoproj/argoexec:${ARGO_WORKFLOWS_VERSION}}"
VEHICLE_ARGOEXEC_IMAGE_TAR="${VEHICLE_ARGOEXEC_IMAGE_TAR:-/tmp/argoexec-${ARGO_WORKFLOWS_VERSION}.arm64.tar}"
VEHICLE_PAUSE_IMAGE="${VEHICLE_PAUSE_IMAGE:-rancher/mirrored-pause:3.6}"
VEHICLE_PAUSE_IMAGE_TAR="${VEHICLE_PAUSE_IMAGE_TAR:-/tmp/rancher-mirrored-pause-3.6.arm64.tar}"
VEHICLE_CLOUD_IMAGE_TAR="${VEHICLE_CLOUD_IMAGE_TAR:-/tmp/vehicle-cloud.amd64.tar}"
ARGOEXEC_IMAGE_TAR="${ARGOEXEC_IMAGE_TAR:-/tmp/argoexec-${ARGO_WORKFLOWS_VERSION}.amd64.tar}"
SKIP_SSH_RUNNER_BUILD="${SKIP_SSH_RUNNER_BUILD:-false}"
SKIP_VEHICLE_RUNNER_BUILD="${SKIP_VEHICLE_RUNNER_BUILD:-false}"
SKIP_VEHICLE_MAPPING_RUNNER_BUILD="${SKIP_VEHICLE_MAPPING_RUNNER_BUILD:-false}"
SSH_IDENTITY_FILE="${SSH_IDENTITY_FILE:-}"

usage() {
  cat <<EOF
Usage:
  ./scripts/shared/build_vehicle_orchestration_images.sh

Environment variables:
  IMAGE_PROFILE             Default: ${IMAGE_PROFILE} (all, autodrive, or mapping)
  SSH_RUNNER_IMAGE           Default: ${SSH_RUNNER_IMAGE}
  SSH_RUNNER_IMAGE_TAR       Default: ${SSH_RUNNER_IMAGE_TAR}
  VEHICLE_RUNNER_IMAGE       Default: ${VEHICLE_RUNNER_IMAGE}
  VEHICLE_RUNNER_IMAGE_TAR   Default: ${VEHICLE_RUNNER_IMAGE_TAR}
  VEHICLE_RUNNER_DOCKERFILE  Default: ${VEHICLE_RUNNER_DOCKERFILE}
  VEHICLE_MAPPING_RUNNER_IMAGE Default: ${VEHICLE_MAPPING_RUNNER_IMAGE}
  VEHICLE_MAPPING_RUNNER_IMAGE_TAR Default: ${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}
  VEHICLE_ARGOEXEC_IMAGE     Default: ${VEHICLE_ARGOEXEC_IMAGE}
  VEHICLE_ARGOEXEC_IMAGE_TAR Default: ${VEHICLE_ARGOEXEC_IMAGE_TAR}
  VEHICLE_PAUSE_IMAGE        Default: ${VEHICLE_PAUSE_IMAGE}
  VEHICLE_PAUSE_IMAGE_TAR    Default: ${VEHICLE_PAUSE_IMAGE_TAR}
  VEHICLE_CLOUD_IMAGE_TAR    Default: ${VEHICLE_CLOUD_IMAGE_TAR}
  ARGOEXEC_IMAGE_TAR         Default: ${ARGOEXEC_IMAGE_TAR} (ARGOEXEC image tar fallback)
  SKIP_SSH_RUNNER_BUILD      Default: ${SKIP_SSH_RUNNER_BUILD}
  SKIP_VEHICLE_RUNNER_BUILD  Default: ${SKIP_VEHICLE_RUNNER_BUILD}
  SKIP_VEHICLE_MAPPING_RUNNER_BUILD Default: ${SKIP_VEHICLE_MAPPING_RUNNER_BUILD}
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

needs_cloud_tar=false
needs_ssh_runner=false
needs_vehicle_runner=false
needs_vehicle_mapping_runner=false
needs_worker_argoexec=false
needs_vehicle_argoexec=false
needs_vehicle_pause=false

case "${IMAGE_PROFILE}" in
  all)
    needs_cloud_tar=true
    needs_ssh_runner=true
    needs_vehicle_runner=true
    needs_vehicle_mapping_runner=true
    needs_worker_argoexec=true
    needs_vehicle_argoexec=true
    needs_vehicle_pause=true
    ;;
  autodrive)
    needs_vehicle_runner=true
    needs_vehicle_argoexec=true
    needs_vehicle_pause=true
    ;;
  mapping)
    needs_cloud_tar=true
    needs_ssh_runner=true
    needs_vehicle_mapping_runner=true
    needs_worker_argoexec=true
    needs_vehicle_argoexec=true
    needs_vehicle_pause=true
    ;;
esac

ensure_platform_image_tar() {
  local image="$1"
  local platform="$2"
  local tar_path="$3"
  local description="$4"

  if [[ -f "${tar_path}" ]]; then
    echo "Reusing ${description} tar: ${tar_path}"
    return 0
  fi

  if ! command -v docker >/dev/null 2>&1; then
    echo "docker is not available and ${description} tar not found: ${tar_path}" >&2
    exit 1
  fi

  docker pull --platform "${platform}" "${image}"
  docker save -o "${tar_path}" "${image}"
}

hash_stdin() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 | awk '{print $1}'
  else
    echo "sha256sum or shasum is required" >&2
    exit 1
  fi
}

hash_repo_files() {
  (
    cd "${ROOT_DIR}"

    for path in "$@"; do
      printf 'FILE %s\n' "${path}"
      cat "${path}"
      printf '\n'
    done
  ) | hash_stdin
}

build_image_hash_input_paths() {
  (
    cd "${ROOT_DIR}"

    if [[ -f ".dockerignore" ]]; then
      printf '%s\n' ".dockerignore"
    fi

    find scripts deploy docker -type f ! -name '.DS_Store' | LC_ALL=C sort

    find config -type f | LC_ALL=C sort | while IFS= read -r path; do
      # Vehicle inventory is synced at deploy time via ConfigMap, not by baking
      # the local workstation's current IP mapping into every runner image.
      if [[ "${path}" == "config/shared/vehicle_hosts.env" ]]; then
        continue
      fi

      printf '%s\n' "${path}"
    done
  )
}

build_ssh_runner_context_hash() {
  local paths=()
  local path=""

  while IFS= read -r path; do
    paths+=("${path}")
  done < <(build_image_hash_input_paths)

  hash_repo_files "${paths[@]}"
}

build_vehicle_runner_context_hash() {
  build_ssh_runner_context_hash
}

docker_image_exists() {
  local image_ref="$1"

  docker image inspect "${image_ref}" >/dev/null 2>&1
}

for cmd in bash python3; do
  require_cmd "${cmd}"
done

if [[ "${needs_cloud_tar}" == "true" && ! -f "${VEHICLE_CLOUD_IMAGE_TAR}" ]]; then
  echo "Required image tar not found: ${VEHICLE_CLOUD_IMAGE_TAR}" >&2
  exit 1
fi

if [[ "${needs_ssh_runner}" == "true" && "${SKIP_SSH_RUNNER_BUILD}" == "true" ]]; then
  if [[ ! -f "${SSH_RUNNER_IMAGE_TAR}" ]]; then
    echo "SKIP_SSH_RUNNER_BUILD=true but ssh-runner tar not found: ${SSH_RUNNER_IMAGE_TAR}" >&2
    exit 1
  fi
elif [[ "${needs_ssh_runner}" == "true" ]] && command -v docker >/dev/null 2>&1; then
  current_ssh_runner_context_hash="$(build_ssh_runner_context_hash)"

  if [[ -f "${SSH_RUNNER_IMAGE_TAR}" && -f "${SSH_RUNNER_BUILD_STATE_FILE}" ]]; then
    previous_ssh_runner_context_hash="$(tr -d '[:space:]' < "${SSH_RUNNER_BUILD_STATE_FILE}")"
  else
    previous_ssh_runner_context_hash=""
  fi

  if [[ "${previous_ssh_runner_context_hash}" == "${current_ssh_runner_context_hash}" ]]; then
    echo "Skipping ssh-runner build; context unchanged: ${current_ssh_runner_context_hash}"
  else
    docker buildx build --platform linux/amd64 -f "${SSH_RUNNER_DOCKERFILE}" -t "${SSH_RUNNER_IMAGE}" "${ROOT_DIR}" --load
    docker save -o "${SSH_RUNNER_IMAGE_TAR}" "${SSH_RUNNER_IMAGE}"
    printf "%s" "${current_ssh_runner_context_hash}" > "${SSH_RUNNER_BUILD_STATE_FILE}"
  fi
elif [[ "${needs_ssh_runner}" == "true" ]]; then
  if [[ ! -f "${SSH_RUNNER_IMAGE_TAR}" ]]; then
    echo "docker is not available and ssh-runner tar not found: ${SSH_RUNNER_IMAGE_TAR}" >&2
    exit 1
  fi
fi

if [[ "${needs_vehicle_runner}" == "true" && "${SKIP_VEHICLE_RUNNER_BUILD}" == "true" ]]; then
  if [[ ! -f "${VEHICLE_RUNNER_IMAGE_TAR}" ]]; then
    echo "SKIP_VEHICLE_RUNNER_BUILD=true but vehicle-runner tar not found: ${VEHICLE_RUNNER_IMAGE_TAR}" >&2
    exit 1
  fi
elif [[ "${needs_vehicle_runner}" == "true" ]] && command -v docker >/dev/null 2>&1; then
  current_vehicle_runner_context_hash="$(build_vehicle_runner_context_hash)"

  if [[ -f "${VEHICLE_RUNNER_IMAGE_TAR}" && -f "${VEHICLE_RUNNER_BUILD_STATE_FILE}" ]]; then
    previous_vehicle_runner_context_hash="$(tr -d '[:space:]' < "${VEHICLE_RUNNER_BUILD_STATE_FILE}")"
  else
    previous_vehicle_runner_context_hash=""
  fi

  if [[ "${previous_vehicle_runner_context_hash}" == "${current_vehicle_runner_context_hash}" ]]; then
    echo "Skipping vehicle-runner build; context unchanged: ${current_vehicle_runner_context_hash}"
  else
    docker buildx build --platform linux/arm64 -f "${VEHICLE_RUNNER_DOCKERFILE}" -t "${VEHICLE_RUNNER_IMAGE}" "${ROOT_DIR}" --load
    docker save -o "${VEHICLE_RUNNER_IMAGE_TAR}" "${VEHICLE_RUNNER_IMAGE}"
    printf "%s" "${current_vehicle_runner_context_hash}" > "${VEHICLE_RUNNER_BUILD_STATE_FILE}"
  fi
elif [[ "${needs_vehicle_runner}" == "true" ]]; then
  if [[ ! -f "${VEHICLE_RUNNER_IMAGE_TAR}" ]]; then
    echo "docker is not available and vehicle-runner tar not found: ${VEHICLE_RUNNER_IMAGE_TAR}" >&2
    exit 1
  fi
fi

if [[ "${needs_vehicle_mapping_runner}" == "true" && "${SKIP_VEHICLE_MAPPING_RUNNER_BUILD}" == "true" ]]; then
  if [[ ! -f "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" ]]; then
    echo "SKIP_VEHICLE_MAPPING_RUNNER_BUILD=true but vehicle-mapping-runner tar not found: ${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" >&2
    exit 1
  fi
elif [[ "${needs_vehicle_mapping_runner}" == "true" ]] && command -v docker >/dev/null 2>&1; then
  current_vehicle_mapping_runner_context_hash="$(build_vehicle_runner_context_hash)"

  if [[ -f "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" && -f "${VEHICLE_MAPPING_RUNNER_BUILD_STATE_FILE}" ]]; then
    previous_vehicle_mapping_runner_context_hash="$(tr -d '[:space:]' < "${VEHICLE_MAPPING_RUNNER_BUILD_STATE_FILE}")"
  else
    previous_vehicle_mapping_runner_context_hash=""
  fi

  if [[ "${previous_vehicle_mapping_runner_context_hash}" == "${current_vehicle_mapping_runner_context_hash}" ]]; then
    echo "Skipping vehicle-mapping-runner build; context unchanged: ${current_vehicle_mapping_runner_context_hash}"
  else
    docker buildx build --platform linux/arm64 -f "${VEHICLE_MAPPING_RUNNER_DOCKERFILE}" -t "${VEHICLE_MAPPING_RUNNER_IMAGE}" "${ROOT_DIR}" --load
    docker save -o "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" "${VEHICLE_MAPPING_RUNNER_IMAGE}"
    printf "%s" "${current_vehicle_mapping_runner_context_hash}" > "${VEHICLE_MAPPING_RUNNER_BUILD_STATE_FILE}"
  fi
elif [[ "${needs_vehicle_mapping_runner}" == "true" ]]; then
  if [[ ! -f "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" ]]; then
    echo "docker is not available and vehicle-mapping-runner tar not found: ${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" >&2
    exit 1
  fi
fi

if [[ "${needs_vehicle_mapping_runner}" == "true" ]]; then
  ensure_platform_image_tar "${VEHICLE_MAPPING_RUNNER_IMAGE}" "linux/arm64" "${VEHICLE_MAPPING_RUNNER_IMAGE_TAR}" "vehicle mapping runner"
fi

if [[ "${needs_worker_argoexec}" == "true" ]]; then
  ensure_platform_image_tar "${VEHICLE_ARGOEXEC_IMAGE}" "linux/amd64" "${ARGOEXEC_IMAGE_TAR}" "worker argoexec"
fi

if [[ "${needs_vehicle_argoexec}" == "true" ]]; then
  ensure_platform_image_tar "${VEHICLE_ARGOEXEC_IMAGE}" "linux/arm64" "${VEHICLE_ARGOEXEC_IMAGE_TAR}" "vehicle argoexec"
fi

if [[ "${needs_vehicle_pause}" == "true" ]]; then
  ensure_platform_image_tar "${VEHICLE_PAUSE_IMAGE}" "linux/arm64" "${VEHICLE_PAUSE_IMAGE_TAR}" "vehicle pause"
fi

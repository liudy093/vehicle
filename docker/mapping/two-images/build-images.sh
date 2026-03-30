#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./docker/mapping/two-images/build-images.sh [docker|nerdctl]

Builds both images:
  - vehicle-cloud:latest
  - vehicle-car:latest

If no tool is provided, the script tries:
  1. docker
  2. nerdctl
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

choose_tool() {
  if [[ -n "${1:-}" ]]; then
    printf '%s\n' "$1"
    return 0
  fi

  if command -v docker >/dev/null 2>&1; then
    printf 'docker\n'
    return 0
  fi

  if command -v nerdctl >/dev/null 2>&1; then
    printf 'nerdctl\n'
    return 0
  fi

  echo "No supported container build tool found. Install docker or nerdctl." >&2
  exit 1
}

TOOL="$(choose_tool "${1:-}")"

if [[ "${TOOL}" != "docker" && "${TOOL}" != "nerdctl" ]]; then
  echo "Unsupported tool: ${TOOL}" >&2
  usage
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

echo "Using build tool: ${TOOL}"
echo "Repository root: ${REPO_ROOT}"

set -x
"${TOOL}" build \
  -f "${REPO_ROOT}/docker/mapping/two-images/cloud/Dockerfile" \
  -t vehicle-cloud:latest \
  "${REPO_ROOT}"

"${TOOL}" build \
  -f "${REPO_ROOT}/docker/mapping/two-images/car/Dockerfile" \
  -t vehicle-car:latest \
  "${REPO_ROOT}"
set +x

echo
echo "Build complete:"
echo "  vehicle-cloud:latest"
echo "  vehicle-car:latest"

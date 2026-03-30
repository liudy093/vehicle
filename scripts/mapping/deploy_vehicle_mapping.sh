#!/usr/bin/env bash
set -euo pipefail

export KUBECONFIG=/tmp/perception.k3s.yaml

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
K8S_DIR="${ROOT_DIR}/deploy/shared/k8s"
ARGO_DIR="${ROOT_DIR}/deploy/mapping/argo"
PREPARE_MAPPING_RUNTIME_SCRIPT="${ROOT_DIR}/scripts/mapping/prepare_vehicle_mapping_runtime.sh"
source "${SCRIPT_DIR}/../shared/lib/common.sh"
source "${SCRIPT_DIR}/../shared/vehicle_inventory.sh"

WORKFLOW_FILE="${WORKFLOW_FILE:-${ARGO_DIR}/vehicle-mapping-workflow.yaml}"
BOOTSTRAP_CLUSTER="${BOOTSTRAP_CLUSTER:-false}"
SSH_IDENTITY_FILE="${SSH_IDENTITY_FILE:-}"
DEFAULT_VEHICLE1_HOST="$(vehicle_inventory_host "vehicle1" "nvidia")"
DEFAULT_VEHICLE2_HOST="$(vehicle_inventory_host "vehicle2" "nvidia")"
DEFAULT_VEHICLE1_HOSTNAME="$(vehicle_inventory_expected_hostname "vehicle1")"
DEFAULT_VEHICLE2_HOSTNAME="$(vehicle_inventory_expected_hostname "vehicle2")"
SSH_OPTS=()
while IFS= read -r opt; do
  SSH_OPTS+=("${opt}")
done < <(build_ssh_opts "${SSH_IDENTITY_FILE}")

usage() {
  cat <<EOF
Usage:
  ./scripts/mapping/deploy_vehicle_mapping.sh

Environment variables:
  WORKFLOW_FILE              Default: ${WORKFLOW_FILE}
  BOOTSTRAP_CLUSTER          Default: ${BOOTSTRAP_CLUSTER}
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
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

collect_online_vehicle_ids() {
  local vehicle_id=""

  for vehicle_id in vehicle1 vehicle2; do
    if vehicle_node_ready "${vehicle_id}"; then
      printf '%s\n' "${vehicle_id}"
    fi
  done
}

prepare_mapping_runtime_on_vehicle() {
  local vehicle_id="$1"
  local vehicle_host=""

  vehicle_host="$(vehicle_inventory_host "${vehicle_id}" "nvidia")"
  ssh "${SSH_OPTS[@]}" "${vehicle_host}" \
    "bash -s" < "${PREPARE_MAPPING_RUNTIME_SCRIPT}"
}

render_pruned_workflow() {
  local source_workflow="$1"
  local rendered_workflow="$2"

  shift 2

  python3 - "${source_workflow}" "${rendered_workflow}" "$@" <<'PY'
import pathlib
import re
import sys


source_path = pathlib.Path(sys.argv[1])
rendered_path = pathlib.Path(sys.argv[2])
online_vehicle_ids = tuple(sys.argv[3:])

root_template_name = "vehicle-mapping"
always_task_names = ["deploy-cloud", "verify-cloud"]
vehicle_task_names = {
    "vehicle1": ["restart-vehicle1", "verify-vehicle1"],
    "vehicle2": ["restart-vehicle2", "verify-vehicle2"],
}
required_task_names = always_task_names + vehicle_task_names["vehicle1"] + vehicle_task_names["vehicle2"]
keep_task_names = list(always_task_names)
for vehicle_id in online_vehicle_ids:
    if vehicle_id not in vehicle_task_names:
        raise SystemExit(f"Unknown vehicle id for workflow pruning: {vehicle_id}")
    keep_task_names.extend(vehicle_task_names[vehicle_id])

required_template_names = [root_template_name] + required_task_names
keep_template_names = [root_template_name] + keep_task_names


def fail(message: str) -> None:
    raise SystemExit(message)


def parse_named_blocks(block_lines, name_pattern, missing_message):
    starts = []
    names = []
    seen = set()
    blocks = {}
    order = []

    for idx, line in enumerate(block_lines):
        match = re.match(name_pattern, line)
        if match:
            name = match.group(1)
            if name in seen:
                fail(f"Duplicate named block found in source workflow: {name}")
            seen.add(name)
            starts.append(idx)
            names.append(name)

    if not starts:
        fail(missing_message)

    for pos, start in enumerate(starts):
        end = starts[pos + 1] if pos + 1 < len(starts) else len(block_lines)
        name = names[pos]
        blocks[name] = block_lines[start:end]
        order.append(name)

    return order, blocks


def prune_root_template_block(block_lines):
    tasks_header_idx = None
    for idx, line in enumerate(block_lines):
        if line.startswith("        tasks:"):
            tasks_header_idx = idx
            break

    if tasks_header_idx is None:
        fail(f"Failed to locate dag tasks in source workflow template: {root_template_name}")

    task_lines = block_lines[tasks_header_idx + 1 :]
    task_order, task_blocks = parse_named_blocks(
        task_lines,
        r'^ {10}- name: ([^#\n]+?)\s*$',
        f"Failed to locate workflow dag tasks in source workflow template: {root_template_name}",
    )

    missing_task_names = [name for name in required_task_names if name not in task_blocks]
    if missing_task_names:
        fail("Source workflow is missing required dag tasks: " + ", ".join(missing_task_names))

    kept_task_lines = []
    keep_task_name_set = set(keep_task_names)
    for name in task_order:
        if name in keep_task_name_set:
            kept_task_lines.extend(task_blocks[name])

    return block_lines[: tasks_header_idx + 1] + kept_task_lines


lines = source_path.read_text().splitlines(keepends=True)
templates_header_idx = None
for idx, line in enumerate(lines):
    if line.startswith("  templates:"):
        templates_header_idx = idx
        break

if templates_header_idx is None:
    fail(f"Failed to locate templates section in source workflow: {source_path}")

template_section_lines = lines[templates_header_idx + 1 :]
template_order, template_blocks = parse_named_blocks(
    template_section_lines,
    r'^ {4}- name: ([^#\n]+?)\s*$',
    f"Failed to locate workflow templates in source workflow: {source_path}",
)

missing_template_names = [name for name in required_template_names if name not in template_blocks]
if missing_template_names:
    fail("Source workflow is missing required templates: " + ", ".join(missing_template_names))

kept_template_lines = []
keep_template_name_set = set(keep_template_names)
for name in template_order:
    if name == root_template_name:
        kept_template_lines.extend(prune_root_template_block(template_blocks[name]))
    elif name in keep_template_name_set:
        kept_template_lines.extend(template_blocks[name])

rendered_path.write_text("".join(lines[: templates_header_idx + 1] + kept_template_lines))
PY
}

wait_for_vehicle_node_ready_window() {
  local node="$1"
  local timeout="${2:-60s}"

  if ! kubectl get node "${node}" >/dev/null 2>&1; then
    return 0
  fi

  kubectl wait --for=condition=Ready "node/${node}" --timeout="${timeout}" >/dev/null 2>&1 || true
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

if [[ "${BOOTSTRAP_CLUSTER}" == "true" ]]; then
  echo "Bootstrapping cluster infrastructure and vehicle agents..."
  bash "${K8S_DIR}/argo-install.sh"
  bash "${K8S_DIR}/register_vehicle1_node.sh"
  bash "${K8S_DIR}/register_vehicle2_node.sh"
else
  echo "Skipping Argo installation and vehicle agent registration. Assuming cluster infrastructure and vehicle nodes already exist."
  APPLY_ARGO_INSTALL_MANIFEST=false bash "${K8S_DIR}/argo-install.sh"
fi

bash "${K8S_DIR}/register_general_workers.sh"
bash "${K8S_DIR}/apply_vehicle_inventory_configmap.sh"

for vehicle_id in vehicle1 vehicle2; do
  wait_for_vehicle_node_ready_window "${vehicle_id}" "60s"
done

kubectl apply -f "${K8S_DIR}/namespace-perception.yaml"

mapfile -t online_vehicle_ids < <(collect_online_vehicle_ids)
if [[ "${#online_vehicle_ids[@]}" -eq 0 ]]; then
  echo "online vehicles: none; deploying cloud-only mapping workflow"
else
  echo "online vehicles: ${online_vehicle_ids[*]}"
  for vehicle_id in "${online_vehicle_ids[@]}"; do
    prepare_mapping_runtime_on_vehicle "${vehicle_id}"
  done
fi

rendered_workflow_file="$(mktemp "${TMPDIR:-/tmp}/vehicle-mapping-pruned.XXXXXX.yaml")"
trap 'rm -f "${rendered_workflow_file}"' EXIT
render_pruned_workflow "${WORKFLOW_FILE}" "${rendered_workflow_file}" "${online_vehicle_ids[@]}"

workflow_name="$(kubectl create -f "${rendered_workflow_file}" -o name)"
echo "Submitted ${workflow_name}"
echo "Watch progress with:"
echo "  kubectl get wf -n argo"
echo "  kubectl get pods -n perception"

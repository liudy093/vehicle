#!/usr/bin/env bash
set -euo pipefail

REMOTE_MAPPING_DIR="${REMOTE_MAPPING_DIR:-/home/nvidia/mapping}"
SOURCE_MAPPING_DIR="${SOURCE_MAPPING_DIR:-/home/nvidia/perception/02mapping}"

mkdir -p "${REMOTE_MAPPING_DIR}"

materialize_workspace() {
  local workspace_name="$1"
  local source_dir="${SOURCE_MAPPING_DIR}/${workspace_name}"
  local runtime_dir="${REMOTE_MAPPING_DIR}/${workspace_name}"
  local staging_dir=""

  if [[ ! -d "${source_dir}" ]]; then
    echo "Missing mapping workspace source: ${source_dir}" >&2
    exit 1
  fi

  staging_dir="$(mktemp -d "${REMOTE_MAPPING_DIR}/.${workspace_name}.XXXXXX")"
  cp -a "${source_dir}" "${staging_dir}/${workspace_name}"
  rm -rf "${runtime_dir}"
  mv "${staging_dir}/${workspace_name}" "${runtime_dir}"
  rmdir "${staging_dir}"
}

materialize_workspace "catkin_ws"
materialize_workspace "ros2_ws"

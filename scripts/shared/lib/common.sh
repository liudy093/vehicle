#!/usr/bin/env bash

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "$1 is required" >&2
    exit 1
  fi
}

build_ssh_opts() {
  local identity_file="${1:-${SSH_IDENTITY_FILE:-}}"
  local -a opts=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR)

  if [[ -n "${identity_file}" && -f "${identity_file}" ]]; then
    opts+=(-i "${identity_file}")
  fi

  printf '%s\n' "${opts[@]}"
}

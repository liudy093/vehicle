#!/usr/bin/env bash

vehicle_host_shell_launcher() {
  cat <<'EOF'
if [ -x /bin/bash ]; then
  exec /bin/bash "$@"
fi
if [ -x /usr/bin/bash ]; then
  exec /usr/bin/bash "$@"
fi
exec /bin/sh "$@"
EOF
}

vehicle_host_target_pid() {
  local candidate=""
  local comm_file=""
  local comm_name=""
  local detected_name=""

  for comm_file in /proc/[0-9]*/comm; do
    [[ -r "${comm_file}" ]] || continue
    candidate="${comm_file#/proc/}"
    candidate="${candidate%/comm}"
    if vehicle_host_root_has_bash "${candidate}" && vehicle_host_root_has_bridge_runtime "${candidate}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  for comm_file in /proc/[0-9]*/comm; do
    [[ -r "${comm_file}" ]] || continue
    candidate="${comm_file#/proc/}"
    candidate="${candidate%/comm}"
    if vehicle_host_root_has_bash "${candidate}" && vehicle_host_root_has_vehicle_runtime_layout "${candidate}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  for comm_name in systemd init; do
    for comm_file in /proc/[0-9]*/comm; do
      [[ -r "${comm_file}" ]] || continue
      candidate="${comm_file#/proc/}"
      candidate="${candidate%/comm}"
      IFS= read -r detected_name < "${comm_file}" || continue
      [[ "${detected_name}" == "${comm_name}" ]] || continue
      if vehicle_host_root_has_bash "${candidate}"; then
        printf '%s\n' "${candidate}"
        return 0
      fi
    done
  done

  for candidate in 1; do
    if vehicle_host_root_has_bash "${candidate}" || vehicle_host_root_has_shell "${candidate}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  echo "unable to resolve a host process for direct vehicle execution" >&2
  return 1
}

vehicle_host_root_has_shell() {
  local pid="$1"

  [[ -x "/proc/${pid}/root/bin/sh" || -x "/proc/${pid}/root/bin/bash" || -x "/proc/${pid}/root/usr/bin/bash" ]]
}

vehicle_host_root_has_bash() {
  local pid="$1"

  [[ -x "/proc/${pid}/root/bin/bash" || -x "/proc/${pid}/root/usr/bin/bash" ]]
}

vehicle_host_root_has_bridge_runtime() {
  local pid="$1"

  [[ -x "/proc/${pid}/root/opt/ros/foxy/lib/ros1_bridge/dynamic_bridge" || \
     -x "/proc/${pid}/root/home/nvidia/mapping/ros2_ws/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge" || \
     -x "/proc/${pid}/root/home/nvidia/ros2_ws/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge" ]]
}

vehicle_host_root_has_vehicle_runtime_layout() {
  local pid="$1"

  [[ -d "/proc/${pid}/root/home/nvidia/mapping" || \
     -d "/proc/${pid}/root/home/nvidia/AutoDrive" ]]
}

vehicle_host_shell_path() {
  local pid="$1"

  if [[ -x "/proc/${pid}/root/bin/bash" ]]; then
    printf '/bin/bash\n'
    return 0
  fi
  if [[ -x "/proc/${pid}/root/usr/bin/bash" ]]; then
    printf '/usr/bin/bash\n'
    return 0
  fi
  printf '/bin/sh\n'
}

vehicle_host_env_exports() {
  local assignment=""
  local name=""
  local value=""

  for assignment in "$@"; do
    name="${assignment%%=*}"
    value="${assignment#*=}"
    printf 'export %s=%q\n' "${name}" "${value}"
  done
  printf 'cd / >/dev/null 2>&1 || true\n'
}

vehicle_host_shell_exec() {
  local shell_path="$1"
  local run_as="${2:-nvidia}"

  if [[ "${run_as}" == "root" ]]; then
    printf 'exec %s -s\n' "${shell_path}"
    return 0
  fi

  cat <<EOF
if command -v sudo >/dev/null 2>&1 && id -u ${run_as} >/dev/null 2>&1; then
  exec sudo -H -u ${run_as} ${shell_path} -s
fi
exec ${shell_path} -s
EOF
}

vehicle_host_nsenter_wd_flag() {
  if nsenter --help 2>&1 | grep -F -- '--wdns' >/dev/null 2>&1; then
    printf -- '--wdns=/\n'
    return 0
  fi

  printf -- '--wd=/\n'
}

vehicle_host_run_with_stdin_as() {
  local run_as="${1:-nvidia}"
  shift || true
  local target_pid=""
  local shell_path=""
  local exec_script=""
  local wd_flag=""

  if ! command -v nsenter >/dev/null 2>&1; then
    echo "nsenter is required for direct vehicle host execution" >&2
    return 1
  fi

  target_pid="$(vehicle_host_target_pid)"
  shell_path="$(vehicle_host_shell_path "${target_pid}")"
  exec_script="$(vehicle_host_shell_exec "${shell_path}" "${run_as}")"
  wd_flag="$(vehicle_host_nsenter_wd_flag)"
  {
    vehicle_host_env_exports "$@"
    cat
  } | nsenter --target "${target_pid}" --mount --uts --ipc --net --pid --root="/proc/${target_pid}/root" "${wd_flag}" -- /bin/sh -c "${exec_script}"
}

run_vehicle_host_script() {
  vehicle_host_run_with_stdin_as nvidia "$@"
}

run_vehicle_host_root_script() {
  vehicle_host_run_with_stdin_as root "$@"
}

run_vehicle_host_command() {
  local command="$1"
  shift || true
  vehicle_host_run_with_stdin_as nvidia "$@" <<EOF
${command}
EOF
}

run_vehicle_host_root_command() {
  local command="$1"
  shift || true
  vehicle_host_run_with_stdin_as root "$@" <<EOF
${command}
EOF
}

#!/usr/bin/env bash

vehicle_autodrive_inventory_dir() {
  cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
}

vehicle_autodrive_source_inventory() {
  # shellcheck disable=SC1091
  source "$(vehicle_autodrive_inventory_dir)/vehicle_inventory.sh"
}

vehicle_autodrive_validate_vehicle_id() {
  local vehicle_id="${1:?vehicle id is required}"

  case "${vehicle_id}" in
    vehicle1|vehicle2)
      ;;
    *)
      echo "Unsupported vehicle id: ${vehicle_id}" >&2
      return 1
      ;;
  esac
}

vehicle_autodrive_host_inventory_key() {
  local vehicle_id="${1:?vehicle id is required}"

  vehicle_autodrive_validate_vehicle_id "${vehicle_id}" || return 1

  case "${vehicle_id}" in
    vehicle1) printf '%s\n' "VEHICLE1_HOST" ;;
    vehicle2) printf '%s\n' "VEHICLE2_HOST" ;;
  esac
}

vehicle_autodrive_host_usage_ref() {
  local vehicle_id="${1:?vehicle id is required}"
  local host_key=""

  host_key="$(vehicle_autodrive_host_inventory_key "${vehicle_id}")" || return 1
  printf '%s\n' "config/shared/vehicle_hosts.env -> ${host_key}"
}

vehicle_autodrive_execution_mode() {
  printf '%s\n' "${VEHICLE_EXECUTION_MODE:-pod}"
}

vehicle_autodrive_require_transport_tools() {
  local execution_mode="${1:-$(vehicle_autodrive_execution_mode)}"

  if [[ "${execution_mode}" == "ssh" ]]; then
    require_cmd expect
    require_cmd ssh
    require_cmd base64
  fi
}

vehicle_autodrive_require_tools() {
  vehicle_autodrive_require_transport_tools "$(vehicle_autodrive_execution_mode)"
}

vehicle_autodrive_run_pod_script() {
  local script_content="${1:?script content is required}"

  bash -s <<EOF
${script_content}
EOF
}

vehicle_autodrive_run_script() {
  local vehicle_id="${1:?vehicle id is required}"
  local host="${2:-}"
  local script_content="${3:?script content is required}"
  local timeout_sec="${4:-180}"

  vehicle_autodrive_validate_vehicle_id "${vehicle_id}" || return 1

  case "$(vehicle_autodrive_execution_mode)" in
    ssh)
      if [[ -z "${host}" ]]; then
        echo "host is required for ssh mode" >&2
        return 1
      fi
      vehicle_autodrive_run_remote_script "${vehicle_id}" "${host}" "${script_content}" "${timeout_sec}"
      ;;
    *)
      vehicle_autodrive_run_pod_script "${script_content}"
      ;;
  esac
}

vehicle_autodrive_default_host() {
  local vehicle_id="${1:?vehicle id is required}"

  vehicle_autodrive_validate_vehicle_id "${vehicle_id}" || return 1
  vehicle_autodrive_source_inventory
  printf '%s\n' "${VEHICLE_HOST:-$(vehicle_inventory_host "${vehicle_id}" "nvidia")}"
}

vehicle_autodrive_expected_hostname() {
  local vehicle_id="${1:?vehicle id is required}"

  vehicle_autodrive_validate_vehicle_id "${vehicle_id}" || return 1
  vehicle_autodrive_source_inventory
  printf '%s\n' "${VEHICLE_EXPECTED_HOSTNAME:-$(vehicle_inventory_expected_hostname "${vehicle_id}")}"
}

vehicle_autodrive_prompt_password() {
  if [[ -n "${VEHICLE_PASSWORD:-}" ]]; then
    export VEHICLE_PASSWORD
    return 0
  fi

  read -r -s -p "SSH password for ${VEHICLE_HOST}: " VEHICLE_PASSWORD
  echo
  export VEHICLE_PASSWORD
}

vehicle_autodrive_endpoint_coords() {
  local endpoint_name="${1:?endpoint name is required}"

  case "${endpoint_name}" in
    end_1) printf '%s\n' "113.67986377 34.58664073" ;;
    end_2) printf '%s\n' "113.67978869 34.58656693" ;;
    end_3) printf '%s\n' "113.67962258 34.58657090" ;;
    *)
      echo "Unsupported endpoint: ${endpoint_name}. Expected one of: end_1, end_2, end_3" >&2
      return 1
      ;;
  esac
}

vehicle_autodrive_remote_command() {
  local host="${1:?host is required}"
  local remote_command="${2:?remote command is required}"
  local timeout_sec="${3:-120}"
  local identity_file="${SSH_IDENTITY_FILE:-}"

  if [[ -n "${identity_file}" && ! -f "${identity_file}" ]]; then
    identity_file=""
  fi

  EXPECT_HOST="${host}" \
  EXPECT_PASSWORD="${VEHICLE_PASSWORD}" \
  EXPECT_REMOTE_COMMAND="${remote_command}" \
  EXPECT_TIMEOUT="${timeout_sec}" \
  EXPECT_IDENTITY_FILE="${identity_file}" \
    expect <<'EOF'
log_user 0

set timeout $env(EXPECT_TIMEOUT)
set host $env(EXPECT_HOST)
set password $env(EXPECT_PASSWORD)
set remote_command $env(EXPECT_REMOTE_COMMAND)

proc shell_quote {value} {
  regsub -all {'} $value {'\\''} quoted
  return "'$quoted'"
}

set ssh_argv [list ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10]
if {[info exists env(EXPECT_IDENTITY_FILE)] && $env(EXPECT_IDENTITY_FILE) ne ""} {
  lappend ssh_argv -i $env(EXPECT_IDENTITY_FILE)
}
lappend ssh_argv $host "bash -lc [shell_quote $remote_command]"

  spawn {*}$ssh_argv

expect {
  -re "(?i)yes/no" {
    send "yes\r"
    exp_continue
  }
  -re "(?i)password:" {
    send "$password\r"
    exp_continue
  }
  timeout {
    puts stderr "SSH command timed out"
    exit 124
  }
  eof
}

set output $expect_out(buffer)
catch wait result
set exit_status [lindex $result 3]
puts -nonewline $output
exit $exit_status
EOF
}

vehicle_autodrive_run_remote_script() {
  local vehicle_id="${1:?vehicle id is required}"
  local host="${2:?host is required}"
  local script_content="${3:?script content is required}"
  local timeout_sec="${4:-180}"
  local encoded_script=""

  vehicle_autodrive_validate_vehicle_id "${vehicle_id}" || return 1

  encoded_script="$(printf '%s' "${script_content}" | base64 | tr -d '\n')"
  vehicle_autodrive_remote_command \
    "${host}" \
    "printf '%s' '${encoded_script}' | base64 -d | bash" \
    "${timeout_sec}"
}

vehicle_autodrive_remote_script() {
  vehicle_autodrive_run_remote_script "$@"
}

vehicle_autodrive_runtime_script() {
  vehicle_autodrive_run_script "$@"
}

vehicle_autodrive_runtime_lib() {
  cat <<'EOF'
source_safe() {
  set +u
  source "$1" >/dev/null 2>&1
  set -u
}

kill_pid_gracefully() {
  local pid="${1:-}"

  if [[ -z "${pid}" ]]; then
    return 0
  fi

  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    return 0
  fi

  kill "${pid}" >/dev/null 2>&1 || true
  sleep 1

  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill -9 "${pid}" >/dev/null 2>&1 || true
  fi
}

stop_arrival_monitor() {
  local self_pid="${1:-}"
  local pid=""

  if [[ -n "${REMOTE_MONITOR_PID_FILE:-}" && -f "${REMOTE_MONITOR_PID_FILE}" ]]; then
    pid="$(tr -dc '0-9' < "${REMOTE_MONITOR_PID_FILE}" || true)"
    if [[ -n "${pid}" && "${pid}" != "${self_pid}" ]]; then
      kill_pid_gracefully "${pid}"
    fi
    rm -f "${REMOTE_MONITOR_PID_FILE}"
  fi

  if [[ -n "${REMOTE_MONITOR_SCRIPT_PATH:-}" ]]; then
    while read -r pid; do
      [[ -z "${pid}" || "${pid}" == "${self_pid}" ]] && continue
      kill_pid_gracefully "${pid}"
    done < <(pgrep -f "${REMOTE_MONITOR_SCRIPT_PATH}" || true)
  fi
}

cleanup_autodrive() {
  local self_pid="${1:-}"

  stop_arrival_monitor "${self_pid}"

  pkill -f 'ros2 launch ./launch/launch_tracker.py' || true
  pkill -f 'ros2 launch ./launch/launch_global_planner.py' || true
  pkill -f 'ros2 launch ./launch/launch_lidar_object.py' || true
  pkill -f 'ros2 launch ./launch/launch_lidar.py' || true
  pkill -f 'ros2 launch ./launch/launch_defaults.py' || true

  pkill -f '/install/car_decision/lib/car_decision/car_decision' || true
  pkill -f '/install/local_path_planning/lib/local_path_planning/local_path_planning' || true
  pkill -f '/install/pid/lib/pid/pid' || true
  pkill -f '/install/car_control/lib/car_control/car_control' || true
  pkill -f '/install/global_path_planning/lib/global_path_planning/global_path_planning' || true
  pkill -f '/install/fusion/lib/fusion/fusion' || true
  pkill -f '/install/gps/lib/gps/gps' || true
  pkill -f '/install/imu/lib/imu/imu' || true
  pkill -f '/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node' || true
  pkill -f '/install/lidar_obstacle/lib/lidar_obstacle/lidar_obstacle' || true
  pkill -f '/install/left_lidar_obstacle/lib/left_lidar_obstacle/left_lidar_obstacle' || true
  pkill -f '/install/right_lidar_obstacle/lib/right_lidar_obstacle/right_lidar_obstacle' || true
  pkill -f '/install/regulator/lib/regulator/regulator' || true
  pkill -f '/install/sonic_obstacle/lib/sonic_obstacle/sonic_obstacle' || true
  pkill -f '/install/net_work/lib/net_work/net_work' || true
  pkill -f '/install/car_ori/lib/car_ori/car_ori' || true
}

reset_ros_daemon() {
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
  sleep 1
}
EOF
}

vehicle_autodrive_remote_lib() {
  vehicle_autodrive_runtime_lib
}

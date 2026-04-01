#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../shared/lib/common.sh"
source "${SCRIPT_DIR}/../shared/lib/vehicle_autodrive_common.sh"

DISPLAY_SCRIPT_PATH="${0}"

usage() {
  local vehicle_id="${1:-<vehicle-id>}"
  local host_ref="config/shared/vehicle_hosts.env -> VEHICLE1_HOST|VEHICLE2_HOST|VEHICLE3_HOST"

  if [[ "${vehicle_id}" == "vehicle1" || "${vehicle_id}" == "vehicle2" || "${vehicle_id}" == "vehicle3" ]]; then
    host_ref="$(vehicle_autodrive_host_usage_ref "${vehicle_id}")"
  fi

  cat <<EOF
Usage:
  ${DISPLAY_SCRIPT_PATH} ${vehicle_id} [end_1|end_2|end_3]

Description:
  Start ${vehicle_id} autodrive on ${vehicle_id} using either:
  - pod mode: run directly inside the scheduled vehicle pod and block until auto-stop completes
  - ssh mode: remote launch over SSH
  Shared runtime defaults:
  - map: visit_1.json
  - default endpoint: end_1
  - mode: full auto-stop after arrival is confirmed
  - known limitation: auto-stop only stops the ROS chain; it does not shift the chassis out of D

Environment variables:
  VEHICLE_HOST         Default: ${host_ref}
  VEHICLE_PASSWORD     SSH password. If unset, the script prompts for it.
  SSH_IDENTITY_FILE    Optional SSH identity file passed to ssh -i
  REMOTE_AUTODRIVE_DIR Default: /home/nvidia/AutoDrive
  VEHICLE_EXECUTION_MODE Default: pod (pod or ssh)
  ARRIVAL_DISTANCE_M   Default: 3.0
  ARRIVAL_SPEED_MPS    Default: 0.35
  ARRIVAL_STABLE_COUNT Default: 6
  ARRIVAL_TIMEOUT_SEC  Default: 1800
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

VEHICLE_ID="${1:-}"
if [[ -z "${VEHICLE_ID}" ]]; then
  usage >&2
  exit 1
fi

vehicle_autodrive_validate_vehicle_id "${VEHICLE_ID}"
shift

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage "${VEHICLE_ID}"
  exit 0
fi

validate_preflight_output() {
  local preflight_output="${1:?preflight output is required}"

  REMOTE_HOSTNAME="$(printf '%s\n' "${preflight_output}" | awk '/^HOSTNAME / {print $2}' | tail -n 1)"
  if [[ -z "${REMOTE_HOSTNAME}" ]]; then
    echo "Preflight missing runtime hostname" >&2
    return 1
  fi

  if [[ "${EXECUTION_MODE}" == "ssh" && "${REMOTE_HOSTNAME}" != "${EXPECTED_HOSTNAME}" ]]; then
    echo "Expected runtime hostname ${EXPECTED_HOSTNAME}, got: ${REMOTE_HOSTNAME}" >&2
    return 1
  fi

  if ! printf '%s\n' "${preflight_output}" | grep -qx 'AUTODRIVE_DIR_OK'; then
    if ! printf '%s\n' "${preflight_output}" | grep -qx 'AUTODRIVE_DIR_MISSING'; then
      echo "Preflight missing autodrive directory status for ${REMOTE_AUTODRIVE_DIR}" >&2
      return 1
    fi
    echo "Required autodrive directory missing under ${REMOTE_AUTODRIVE_DIR}" >&2
    return 1
  fi

  if ! printf '%s\n' "${preflight_output}" | grep -qx 'MAP_OK'; then
    if ! printf '%s\n' "${preflight_output}" | grep -qx 'MAP_MISSING'; then
      echo "Preflight missing map status for ${REMOTE_MAP_PATH}" >&2
      return 1
    fi
    echo "Required autodrive map missing under ${REMOTE_MAP_PATH}" >&2
    return 1
  fi
}

ENDPOINT_NAME="${1:-end_1}"
read -r END_LON END_LAT <<<"$(vehicle_autodrive_endpoint_coords "${ENDPOINT_NAME}")"

VEHICLE_EXECUTION_MODE="${VEHICLE_EXECUTION_MODE:-pod}"
EXECUTION_MODE="$(vehicle_autodrive_execution_mode)"

vehicle_autodrive_require_transport_tools "${EXECUTION_MODE}"

REMOTE_AUTODRIVE_DIR="${REMOTE_AUTODRIVE_DIR:-/home/nvidia/AutoDrive}"
REMOTE_LOG_DIR="${REMOTE_AUTODRIVE_DIR}/datas/logs/remote_start"
REMOTE_MAP_PATH="${REMOTE_AUTODRIVE_DIR}/datas/maps/visit_1.json"
ARRIVAL_DISTANCE_M="${ARRIVAL_DISTANCE_M:-3.0}"
ARRIVAL_SPEED_MPS="${ARRIVAL_SPEED_MPS:-0.35}"
ARRIVAL_STABLE_COUNT="${ARRIVAL_STABLE_COUNT:-6}"
ARRIVAL_TIMEOUT_SEC="${ARRIVAL_TIMEOUT_SEC:-1800}"

RUNTIME_WAIT_MODE="blocking"
VEHICLE_HOST="${VEHICLE_HOST:-}"
EXPECTED_HOSTNAME="${EXPECTED_HOSTNAME:-}"
if [[ "${EXECUTION_MODE}" == "ssh" ]]; then
  VEHICLE_HOST="${VEHICLE_HOST:-$(vehicle_autodrive_default_host "${VEHICLE_ID}")}"
  EXPECTED_HOSTNAME="${EXPECTED_HOSTNAME:-$(vehicle_autodrive_expected_hostname "${VEHICLE_ID}")}"
  RUNTIME_WAIT_MODE="background"
  vehicle_autodrive_prompt_password
fi

HOST_PREFLIGHT_SCRIPT="$(cat <<EOF
set -euo pipefail

REMOTE_AUTODRIVE_DIR='${REMOTE_AUTODRIVE_DIR}'
REMOTE_MAP_PATH='${REMOTE_MAP_PATH}'

echo "HOSTNAME \$(hostname)"
if [[ -d "\${REMOTE_AUTODRIVE_DIR}" ]]; then
  echo "AUTODRIVE_DIR_OK"
else
  echo "AUTODRIVE_DIR_MISSING"
fi

if [[ -f "\${REMOTE_MAP_PATH}" ]]; then
  echo "MAP_OK"
else
  echo "MAP_MISSING"
fi
EOF
)"

REMOTE_SCRIPT="$(cat <<EOF
set -euo pipefail

REMOTE_AUTODRIVE_DIR='${REMOTE_AUTODRIVE_DIR}'
REMOTE_LOG_DIR='${REMOTE_LOG_DIR}'
REMOTE_MAP_PATH='${REMOTE_MAP_PATH}'
REMOTE_MONITOR_SCRIPT_PATH='${REMOTE_LOG_DIR}/arrival_monitor.sh'
REMOTE_MONITOR_LOG_PATH='${REMOTE_LOG_DIR}/arrival_monitor.log'
REMOTE_MONITOR_PID_FILE='${REMOTE_LOG_DIR}/arrival_monitor.pid'
ENDPOINT_NAME='${ENDPOINT_NAME}'
END_LON='${END_LON}'
END_LAT='${END_LAT}'
ARRIVAL_DISTANCE_M='${ARRIVAL_DISTANCE_M}'
ARRIVAL_SPEED_MPS='${ARRIVAL_SPEED_MPS}'
ARRIVAL_STABLE_COUNT='${ARRIVAL_STABLE_COUNT}'
ARRIVAL_TIMEOUT_SEC='${ARRIVAL_TIMEOUT_SEC}'
RUNTIME_WAIT_MODE='${RUNTIME_WAIT_MODE}'

mkdir -p "\${REMOTE_LOG_DIR}"

$(vehicle_autodrive_remote_lib)

BOOT_TS="\$(date +%s)"
AUTODRIVE_PROCESS_PATTERN='arrival_monitor.sh|launch_tracker.py|launch_global_planner.py|launch_lidar_object.py|launch_lidar.py|launch_defaults.py|/install/car_decision/lib/car_decision/car_decision|/install/local_path_planning/lib/local_path_planning/local_path_planning|/install/pid/lib/pid/pid|/install/car_control/lib/car_control/car_control|/install/global_path_planning/lib/global_path_planning/global_path_planning|/install/fusion/lib/fusion/fusion|/install/gps/lib/gps/gps|/install/imu/lib/imu/imu|/install/rslidar_sdk/lib/rslidar_sdk/rslidar_sdk_node|/install/lidar_obstacle/lib/lidar_obstacle/lidar_obstacle|/install/left_lidar_obstacle/lib/left_lidar_obstacle/left_lidar_obstacle|/install/right_lidar_obstacle/lib/right_lidar_obstacle/right_lidar_obstacle|/install/regulator/lib/regulator/regulator|/install/sonic_obstacle/lib/sonic_obstacle/sonic_obstacle|/install/net_work/lib/net_work/net_work|/install/car_ori/lib/car_ori/car_ori'
AUTODRIVE_STARTED=0
KEEP_RUNNING_ON_EXIT=0

phase_log() {
  local phase="\$1"
  local now_ts elapsed_sec
  now_ts="\$(date +%s)"
  elapsed_sec="\$((now_ts - BOOT_TS))"
  echo "PHASE t=+\${elapsed_sec}s \${phase}"
}

launch_bg() {
  local name="\$1"
  local command="\$2"
  nohup bash -lc "\${command}" >"\${REMOTE_LOG_DIR}/\${name}.log" 2>&1 < /dev/null &
  echo "STARTED \${name} pid=\$!"
}

wait_until_no_autodrive_processes() {
  local timeout_sec="\${1:-8}"
  local deadline="\$((SECONDS + timeout_sec))"
  while (( SECONDS < deadline )); do
    if ! pgrep -af "\${AUTODRIVE_PROCESS_PATTERN}" >/dev/null; then
      return 0
    fi
    sleep 0.2
  done

  echo "WARN cleanup_timeout"
  pgrep -af "\${AUTODRIVE_PROCESS_PATTERN}" || true
}

get_valid_fusion_coords() {
  python3 - <<'PY'
import rclpy
from rclpy.node import Node
from car_interfaces.msg import FusionInterface

class Grabber(Node):
    def __init__(self):
        super().__init__('fusion_grabber')
        self.msg = None
        self.create_subscription(FusionInterface, '/fusion_data', self.cb, 10)

    def cb(self, msg):
        self.msg = msg

rclpy.init()
node = Grabber()
end_ns = node.get_clock().now().nanoseconds + 30_000_000_000
while rclpy.ok() and node.get_clock().now().nanoseconds < end_ns:
    rclpy.spin_once(node, timeout_sec=0.5)
    if node.msg is None:
        continue
    if abs(node.msg.longitude) > 1e-6 and abs(node.msg.latitude) > 1e-6:
        print(node.msg.longitude, node.msg.latitude)
        break
else:
    raise SystemExit('未获取到有效 /fusion_data 经纬度')

node.destroy_node()
rclpy.shutdown()
PY
}

publish_start_end_once() {
  ros2 topic pub --once /hmi_start_end_point_data car_interfaces/msg/HmiStartEndPointInterface \
    "{timestamp: 0.0, startpoint: [\${START_LON}, \${START_LAT}], endpoint: [\${END_LON}, \${END_LAT}], process_time: 0.0}" >/dev/null
}

wait_for_route_ready() {
  local timeout_sec="\${1:-6}"
  python3 - "\${timeout_sec}" <<'PY'
import sys

import rclpy
from rclpy.node import Node

from car_interfaces.msg import GlobalPathPlanningInterface


class Grabber(Node):
    def __init__(self):
        super().__init__('global_path_grabber')
        self.msg = None
        self.create_subscription(GlobalPathPlanningInterface, '/global_path_planning_data', self.cb, 10)

    def cb(self, msg):
        self.msg = msg


timeout_sec = float(sys.argv[1])
rclpy.init()
node = Grabber()
end_ns = node.get_clock().now().nanoseconds + int(timeout_sec * 1_000_000_000)

while rclpy.ok() and node.get_clock().now().nanoseconds < end_ns:
    rclpy.spin_once(node, timeout_sec=0.5)
    if node.msg is None:
        continue
    if len(node.msg.routedata) > 6:
        print(len(node.msg.routedata))
        break
else:
    raise SystemExit('未获取到有效 /global_path_planning_data')

node.destroy_node()
rclpy.shutdown()
PY
}

wait_for_tracker_processes() {
  local timeout_sec="\${1:-8}"
  local deadline="\$((SECONDS + timeout_sec))"
  local missing=""
  while (( SECONDS < deadline )); do
    missing=""
    for process_name in car_decision local_path_planning pid car_control; do
      if ! pgrep -af "\${process_name}" >/dev/null; then
        missing="\${missing} \${process_name}"
      fi
    done

    if [[ -z "\${missing}" ]]; then
      return 0
    fi

    sleep 0.2
  done

  echo "ERROR missing_process\${missing}"
  return 1
}

wait_for_stable_processes() {
  local timeout_sec="\${1:-5}"
  local deadline="\$((SECONDS + timeout_sec))"
  local missing=""

  while (( SECONDS < deadline )); do
    missing=""
    for process_name in car_decision local_path_planning pid car_control fusion gps; do
      if ! pgrep -af "\${process_name}" >/dev/null; then
        missing="\${missing} \${process_name}"
      fi
    done

    if [[ -n "\${missing}" ]]; then
      echo "ERROR unstable_processes\${missing}"
      return 1
    fi

    sleep 0.5
  done

  return 0
}

cleanup_on_launch_failure() {
  local exit_code="\${1:-0}"

  if [[ "\${exit_code}" -eq 0 || "\${AUTODRIVE_STARTED}" -ne 1 || "\${KEEP_RUNNING_ON_EXIT}" -eq 1 ]]; then
    return 0
  fi

  echo "LAUNCH_FAILURE_CLEANUP code=\${exit_code}"
  cleanup_autodrive "\${BASHPID}"
  wait_until_no_autodrive_processes 8 || true
  reset_ros_daemon
}

on_script_exit() {
  local exit_code="\$?"
  trap - EXIT
  cleanup_on_launch_failure "\${exit_code}"
  exit "\${exit_code}"
}

trap on_script_exit EXIT

write_arrival_monitor() {
  cat > "\${REMOTE_MONITOR_SCRIPT_PATH}" <<'MONITOR'
#!/usr/bin/env bash
set -euo pipefail

REMOTE_AUTODRIVE_DIR='${REMOTE_AUTODRIVE_DIR}'
REMOTE_LOG_DIR='${REMOTE_LOG_DIR}'
REMOTE_MONITOR_SCRIPT_PATH='${REMOTE_LOG_DIR}/arrival_monitor.sh'
REMOTE_MONITOR_PID_FILE='${REMOTE_LOG_DIR}/arrival_monitor.pid'
ENDPOINT_NAME='${ENDPOINT_NAME}'
END_LON='${END_LON}'
END_LAT='${END_LAT}'
ARRIVAL_DISTANCE_M='${ARRIVAL_DISTANCE_M}'
ARRIVAL_SPEED_MPS='${ARRIVAL_SPEED_MPS}'
ARRIVAL_STABLE_COUNT='${ARRIVAL_STABLE_COUNT}'
ARRIVAL_TIMEOUT_SEC='${ARRIVAL_TIMEOUT_SEC}'

$(vehicle_autodrive_remote_lib)

cd "\${REMOTE_AUTODRIVE_DIR}"
source_safe /opt/ros/foxy/setup.bash
source_safe "\${REMOTE_AUTODRIVE_DIR}/install/setup.bash"

echo "ARRIVAL_MONITOR_START endpoint=\${ENDPOINT_NAME} lon=\${END_LON} lat=\${END_LAT} distance_m=\${ARRIVAL_DISTANCE_M} speed_mps=\${ARRIVAL_SPEED_MPS} stable_count=\${ARRIVAL_STABLE_COUNT}"

set +e
python3 - "\${END_LON}" "\${END_LAT}" "\${ARRIVAL_DISTANCE_M}" "\${ARRIVAL_SPEED_MPS}" "\${ARRIVAL_STABLE_COUNT}" "\${ARRIVAL_TIMEOUT_SEC}" <<'PY'
import math
import sys

import rclpy
from rclpy.node import Node

from car_interfaces.msg import FusionInterface


def haversine_m(lon1, lat1, lon2, lat2):
    radius = 6371000.0
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * radius * math.asin(math.sqrt(a))


class ArrivalMonitor(Node):
    def __init__(self):
        super().__init__('arrival_monitor')
        self.msg = None
        self.create_subscription(FusionInterface, '/fusion_data', self._cb, 10)

    def _cb(self, msg):
        self.msg = msg


end_lon = float(sys.argv[1])
end_lat = float(sys.argv[2])
distance_threshold = float(sys.argv[3])
speed_threshold = float(sys.argv[4])
stable_required = int(sys.argv[5])
timeout_sec = int(sys.argv[6])

rclpy.init()
node = ArrivalMonitor()
deadline_ns = node.get_clock().now().nanoseconds + timeout_sec * 1_000_000_000
stable_hits = 0

try:
    while rclpy.ok() and node.get_clock().now().nanoseconds < deadline_ns:
        rclpy.spin_once(node, timeout_sec=0.5)
        msg = node.msg
        if msg is None:
            continue
        if abs(msg.longitude) <= 1e-6 or abs(msg.latitude) <= 1e-6:
            continue

        distance_m = haversine_m(msg.longitude, msg.latitude, end_lon, end_lat)
        speed_mps = float(msg.carspeed)

        if distance_m <= distance_threshold and speed_mps <= speed_threshold:
            stable_hits += 1
            print(f"ARRIVAL_WINDOW hit={stable_hits}/{stable_required} distance_m={distance_m:.2f} speed_mps={speed_mps:.2f}", flush=True)
        else:
            if stable_hits:
                print(f"ARRIVAL_WINDOW_RESET distance_m={distance_m:.2f} speed_mps={speed_mps:.2f}", flush=True)
            stable_hits = 0

        if stable_hits >= stable_required:
            print(f"ARRIVAL_CONFIRMED distance_m={distance_m:.2f} speed_mps={speed_mps:.2f}", flush=True)
            raise SystemExit(0)

    print(f"ARRIVAL_TIMEOUT timeout_sec={timeout_sec}", flush=True)
    raise SystemExit(124)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
monitor_status=$?
set -e

rm -f "\${REMOTE_MONITOR_PID_FILE}"

if [[ "\${monitor_status}" -eq 0 ]]; then
  echo "AUTO_STOP_TRIGGERED endpoint=\${ENDPOINT_NAME}"
  cleanup_autodrive "\${BASHPID}"
  reset_ros_daemon
  echo "AUTO_STOP_COMPLETED endpoint=\${ENDPOINT_NAME}"
  exit 0
fi

echo "ARRIVAL_MONITOR_EXIT code=\${monitor_status}"
exit "\${monitor_status}"
MONITOR

  chmod +x "\${REMOTE_MONITOR_SCRIPT_PATH}"
}

start_arrival_monitor() {
  write_arrival_monitor
  nohup "\${REMOTE_MONITOR_SCRIPT_PATH}" >"\${REMOTE_MONITOR_LOG_PATH}" 2>&1 < /dev/null &
  printf '%s\n' "\$!" > "\${REMOTE_MONITOR_PID_FILE}"
  echo "MONITOR_READY pid=\$(cat "\${REMOTE_MONITOR_PID_FILE}") log=\${REMOTE_MONITOR_LOG_PATH}"
}

run_arrival_monitor_blocking() {
  write_arrival_monitor
  "\${REMOTE_MONITOR_SCRIPT_PATH}" > >(tee "\${REMOTE_MONITOR_LOG_PATH}") 2>&1 &
  local monitor_pid="\$!"
  printf '%s\n' "\${monitor_pid}" > "\${REMOTE_MONITOR_PID_FILE}"
  echo "MONITOR_READY pid=\${monitor_pid} log=\${REMOTE_MONITOR_LOG_PATH}"
  wait "\${monitor_pid}"
}

cd "\${REMOTE_AUTODRIVE_DIR}"
source_safe /opt/ros/foxy/setup.bash
source_safe "\${REMOTE_AUTODRIVE_DIR}/install/setup.bash"

if [[ ! -f "\${REMOTE_MAP_PATH}" ]]; then
  echo "Map missing: \${REMOTE_MAP_PATH}" >&2
  exit 1
fi

phase_log cleanup_begin
cleanup_autodrive
wait_until_no_autodrive_processes 8
phase_log cleanup_done

AUTODRIVE_STARTED=1
phase_log launch_base_begin
launch_bg launch_defaults "cd '\${REMOTE_AUTODRIVE_DIR}' && source /opt/ros/foxy/setup.bash >/dev/null 2>&1 && source '\${REMOTE_AUTODRIVE_DIR}/install/setup.bash' >/dev/null 2>&1 && exec ros2 launch ./launch/launch_defaults.py"
launch_bg launch_lidar "cd '\${REMOTE_AUTODRIVE_DIR}' && source /opt/ros/foxy/setup.bash >/dev/null 2>&1 && source '\${REMOTE_AUTODRIVE_DIR}/install/setup.bash' >/dev/null 2>&1 && exec ros2 launch ./launch/launch_lidar.py"
launch_bg launch_lidar_object "cd '\${REMOTE_AUTODRIVE_DIR}' && source /opt/ros/foxy/setup.bash >/dev/null 2>&1 && source '\${REMOTE_AUTODRIVE_DIR}/install/setup.bash' >/dev/null 2>&1 && exec ros2 launch ./launch/launch_lidar_object.py"
launch_bg launch_global_planner "cd '\${REMOTE_AUTODRIVE_DIR}' && source /opt/ros/foxy/setup.bash >/dev/null 2>&1 && source '\${REMOTE_AUTODRIVE_DIR}/install/setup.bash' >/dev/null 2>&1 && exec ros2 launch ./launch/launch_global_planner.py use_trajectory:=False file_path:=\${REMOTE_MAP_PATH}"
phase_log launch_base_done

phase_log wait_fusion_begin
read -r START_LON START_LAT < <(get_valid_fusion_coords)
echo "FUSION_START \${START_LON} \${START_LAT}"
phase_log wait_fusion_done

phase_log route_publish_begin
publish_start_end_once
if ROUTE_FIELDS="\$(wait_for_route_ready 4)"; then
  :
else
  phase_log route_publish_retry
  publish_start_end_once
  ROUTE_FIELDS="\$(wait_for_route_ready 8)"
fi
echo "ROUTE_READY fields=\${ROUTE_FIELDS}"
phase_log route_ready

phase_log tracker_launch_begin
launch_bg launch_tracker "cd '\${REMOTE_AUTODRIVE_DIR}' && source /opt/ros/foxy/setup.bash >/dev/null 2>&1 && source '\${REMOTE_AUTODRIVE_DIR}/install/setup.bash' >/dev/null 2>&1 && exec ros2 launch ./launch/launch_tracker.py"
wait_for_tracker_processes 8
phase_log tracker_stability_begin
wait_for_stable_processes 5
phase_log tracker_stability_done
phase_log tracker_ready

if [[ "\${RUNTIME_WAIT_MODE}" == "blocking" ]]; then
  echo "TRACKER_READY endpoint=\${ENDPOINT_NAME} lon=\${END_LON} lat=\${END_LAT}"
  echo "LOG_DIR \${REMOTE_LOG_DIR}"
  run_arrival_monitor_blocking
else
  start_arrival_monitor
  KEEP_RUNNING_ON_EXIT=1
  echo "TRACKER_READY endpoint=\${ENDPOINT_NAME} lon=\${END_LON} lat=\${END_LAT}"
  echo "LOG_DIR \${REMOTE_LOG_DIR}"
fi
EOF
)"

PREFLIGHT_OUTPUT="$(vehicle_autodrive_run_script "${VEHICLE_ID}" "${VEHICLE_HOST}" "${HOST_PREFLIGHT_SCRIPT}" 30)"
validate_preflight_output "${PREFLIGHT_OUTPUT}"

case "${EXECUTION_MODE}" in
  ssh)
    echo "Starting ${VEHICLE_ID} autodrive on ${VEHICLE_HOST} using ${ENDPOINT_NAME}..."
    ;;
  *)
    echo "Starting ${VEHICLE_ID} autodrive inside the vehicle pod using ${ENDPOINT_NAME}..."
    ;;
esac

vehicle_autodrive_run_script "${VEHICLE_ID}" "${VEHICLE_HOST}" "${REMOTE_SCRIPT}" 240

case "${EXECUTION_MODE}" in
  ssh)
    cat <<EOF
${VEHICLE_ID} autodrive start requested successfully.
Remote host: ${VEHICLE_HOST} (${REMOTE_HOSTNAME})
Map:         ${REMOTE_MAP_PATH}
Endpoint:    ${ENDPOINT_NAME} -> [${END_LON}, ${END_LAT}]
Logs:        ${REMOTE_LOG_DIR}
Auto-stop:   enabled (monitor log: ${REMOTE_LOG_DIR}/arrival_monitor.log)
EOF
    ;;
  *)
    cat <<EOF
${VEHICLE_ID} autodrive completed successfully.
Runtime host: ${REMOTE_HOSTNAME}
Map:         ${REMOTE_MAP_PATH}
Endpoint:    ${ENDPOINT_NAME} -> [${END_LON}, ${END_LAT}]
Logs:        ${REMOTE_LOG_DIR}
Auto-stop:   completed
EOF
    ;;
esac

#!/usr/bin/env bash
# Robot-side launcher for tianyee UDP bridge (called by systemd).
set +u
set -o pipefail

INSTALL_DIR="${TIANYEE_BRIDGE_HOME:-/home/ubuntu/pico_vr_teleop_tianyee}"
LOG_DIR="${TIANYEE_BRIDGE_LOG:-/home/ubuntu/pico_vr_teleop_tianyee/logs}"
STATUS_FILE="${TIANYEE_BRIDGE_STATUS:-/home/ubuntu/pico_vr_teleop_tianyee/status.json}"
UDP_PORT="${TIANYEE_UDP_PORT:-19011}"
WAIT_ENABLE_S="${TIANYEE_WAIT_ENABLE_S:-300}"
WAIT_ARM_OK_S="${TIANYEE_WAIT_ARM_OK_S:-120}"
# Do NOT auto --prepare on boot: it fights official proc_manager self-check/TTS.
# PC teleop / go_home_joints will enable as needed.
DO_PREPARE="${TIANYEE_BRIDGE_PREPARE:-0}"

mkdir -p "$LOG_DIR" /tmp/xarm_run/ros_home
export ROS_HOME=/tmp/xarm_run/ros_home
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/data/param/dds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONUNBUFFERED=1
export PYTHONPATH="${INSTALL_DIR}/backends/tianyee:${INSTALL_DIR}/common:${INSTALL_DIR}:${PYTHONPATH:-}"
export TIANYEE_BRIDGE_STATUS_FILE="$STATUS_FILE"

source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2ws/install/setup.bash
source /home/ubuntu/XARM/install/setup.bash

log() { echo "[tianyee-bridge] $*"; }

have_set_arm_enable() {
  # Boot must not depend on DDS discovery (rclpy/ros2 CLI often hang here).
  # Once tianyee_xarm is up, start the UDP bridge; teleop `prepare` enables arms.
  systemctl is-active --quiet tianyee_xarm.service 2>/dev/null
}

wait_set_arm_enable() {
  local i=0
  local max=$((WAIT_ENABLE_S / 5))
  [[ "$max" -lt 1 ]] && max=1
  while [[ "$i" -lt "$max" ]]; do
    if have_set_arm_enable; then
      log "SERVICE_OK tianyee_xarm active (skip DDS probe; prepare later)"
      return 0
    fi
    i=$((i + 1))
    log "wait tianyee_xarm ($i/$max) ..."
    sleep 5
  done
  log "ERROR: tianyee_xarm not active after ${WAIT_ENABLE_S}s"
  return 1
}

wait_arm_online() {
  # Soft wait only: hung ros2 CLI previously blocked boot forever.
  # Bridge status monitor will keep reporting motor health after listen.
  local i=0
  local max=6
  while [[ "$i" -lt "$max" ]]; do
    if systemctl is-active --quiet tianyee_xarm.service 2>/dev/null; then
      log "ARM_STACK_OK tianyee_xarm active — continue"
      return 0
    fi
    i=$((i + 1))
    log "wait tianyee_xarm ($i/$max) ..."
    sleep 5
  done
  log "WARN: tianyee_xarm not active — starting bridge anyway"
  return 0
}

log "install=$INSTALL_DIR port=$UDP_PORT prepare=$DO_PREPARE"
if [[ ! -f "$INSTALL_DIR/backends/tianyee/udp_ros_bridge.py" ]]; then
  log "ERROR: bridge code missing under $INSTALL_DIR"
  exit 10
fi

pkill -f 'backends/tianyee/udp_ros_bridge.py' 2>/dev/null || true
sleep 0.5

wait_set_arm_enable || exit 20
wait_arm_online

prep_args=(--udp-port "$UDP_PORT" --no-disable-on-exit --status-file "$STATUS_FILE" --status-period-s 2.0)
if [[ "$DO_PREPARE" == "1" || "$DO_PREPARE" == "true" ]]; then
  prep_args+=(--prepare)
  log "starting with --prepare (TIANYEE_BRIDGE_PREPARE=1)"
else
  log "starting WITHOUT --prepare (avoid fighting official self-check)"
fi

exec python3 "$INSTALL_DIR/backends/tianyee/udp_ros_bridge.py" "${prep_args[@]}"

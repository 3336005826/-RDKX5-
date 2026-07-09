#!/usr/bin/env bash
set -e

GUI_PID=""
MAP_RELAY_PID=""
CLEANED_UP=0

cleanup() {
  if [ "${CLEANED_UP}" = "1" ]; then
    return
  fi
  CLEANED_UP=1

  if [ -n "${GUI_PID}" ] && kill -0 "${GUI_PID}" >/dev/null 2>&1; then
    kill -INT "${GUI_PID}" >/dev/null 2>&1 || true
    wait "${GUI_PID}" >/dev/null 2>&1 || true
  fi

  if [ -n "${MAP_RELAY_PID}" ] && kill -0 "${MAP_RELAY_PID}" >/dev/null 2>&1; then
    kill -INT "${MAP_RELAY_PID}" >/dev/null 2>&1 || true
    sleep 0.3
    kill "${MAP_RELAY_PID}" >/dev/null 2>&1 || true
  fi
}

trap cleanup INT TERM EXIT

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [ -f "/opt/ros/humble/setup.bash" ]; then
  source /opt/ros/humble/setup.bash
fi

if [ -n "${1:-}" ] && [ -f "$1/install/setup.bash" ]; then
  source "$1/install/setup.bash"
elif [ -f "${HOME}/test_ws/install/setup.bash" ]; then
  source "${HOME}/test_ws/install/setup.bash"
fi

python3 "${SCRIPT_DIR}/station_gui.py" &
GUI_PID=$!

if ros2 pkg executables car_mission_system 2>/dev/null | grep -q "station_map_relay.py"; then
  ros2 run car_mission_system station_map_relay.py &
  MAP_RELAY_PID=$!
fi

wait "${GUI_PID}"

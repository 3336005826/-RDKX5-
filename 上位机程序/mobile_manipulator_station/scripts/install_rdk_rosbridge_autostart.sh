#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="rosbridge-websocket.service"
START_SCRIPT="/usr/local/bin/start_rosbridge_websocket.sh"
WORKSPACE="${1:-/home/sunrise/test_ws}"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID_VALUE:-30}"

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required."
  exit 1
fi

if [ ! -f "/opt/ros/humble/setup.bash" ]; then
  echo "Missing /opt/ros/humble/setup.bash"
  exit 1
fi

if [ ! -f "${WORKSPACE}/install/setup.bash" ]; then
  echo "Missing ${WORKSPACE}/install/setup.bash"
  echo "Usage: $0 /home/sunrise/test_ws"
  exit 1
fi

sudo tee "${START_SCRIPT}" >/dev/null <<EOF
#!/usr/bin/env bash
set -e

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
export ROS_LOCALHOST_ONLY=0
export RCUTILS_LOGGING_BUFFERED_STREAM=1

source /opt/ros/humble/setup.bash
source ${WORKSPACE}/install/setup.bash

exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
EOF

sudo chmod +x "${START_SCRIPT}"

sudo tee "/etc/systemd/system/${SERVICE_NAME}" >/dev/null <<EOF
[Unit]
Description=ROS2 rosbridge websocket for mobile manipulator station
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=sunrise
WorkingDirectory=${WORKSPACE}
ExecStart=${START_SCRIPT}
Restart=always
RestartSec=3
KillSignal=SIGINT
TimeoutStopSec=15

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

echo "Installed and started ${SERVICE_NAME}"
echo "Check status with:"
echo "  systemctl status ${SERVICE_NAME}"
echo "  journalctl -u ${SERVICE_NAME} -f"

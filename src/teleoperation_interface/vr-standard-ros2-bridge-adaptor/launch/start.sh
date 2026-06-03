#!/usr/bin/env bash
# VR-Standard ROS2 Bridge Adaptor — Launch Script
#
# Starts the HTTPS bridge server. VR device connects to the displayed URL.
# Uses python3 from PATH — activate your env (conda/venv) before running.
#
# Usage:
#   bash launch/start.sh                          # default (config.json: port 5100, HTTPS)
#   bash launch/start.sh --port 5002 --no-ssl     # HTTP over ADB
#   bash launch/start.sh --host <host-ip>          # LAN access (auto-generates cert)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

set -u

echo "============================================"
echo "  VR-Standard ROS2 Bridge Adaptor"
echo "============================================"
echo ""

PORT=$(python3 -c "import json; print(json.load(open('config/config.json')).get('port',5100))" 2>/dev/null || echo 5100)

echo "  Starting VR bridge server..."
echo "  VR device browser: https://127.0.0.1:$PORT"
echo "  ROS2 topics: /vr/head_pose, /vr/left_pose, /vr/right_pose,"
echo "               /vr/left_buttons, /vr/right_buttons, /vr_raw_data"
echo ""

exec python3 vr_bridge_server.py "$@"

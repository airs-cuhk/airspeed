#!/usr/bin/env bash
# VR-Standard ROS2 Bridge Adaptor — Launch Script
#
# Starts the HTTPS bridge server. VR device connects to the displayed URL.
#
# Usage:
#   bash launch/start.sh                          # default (config.json: port 5100, HTTPS)
#   bash launch/start.sh --port 5002 --no-ssl     # HTTP over ADB
#   bash launch/start.sh --host <host-ip>          # LAN access (auto-generates cert)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT"

# ---------------------------------------------------------------------------
# Prerequisites (before set -u — ROS2 scripts reference unbound vars)
# ---------------------------------------------------------------------------

if [ -n "${PYTHON_BIN:-}" ]; then
    PYTHON="$PYTHON_BIN"
else
    PYTHON=""
    for py in /usr/bin/python3.10 python3.10 python3; do
        if command -v "$py" &>/dev/null; then
            PYTHON="$py"; break
        fi
    done
fi
if [ -z "$PYTHON" ]; then
    echo "ERROR: Python 3.10+ not found. Set python: in global_config.yaml or install python3.10."
    exit 1
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

set -u

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

echo "============================================"
echo "  VR-Standard ROS2 Bridge Adaptor"
echo "============================================"
echo ""

# Read default port from config if not overridden by CLI
PORT=$("$PYTHON" -c "import json; print(json.load(open('config/config.json')).get('port',5100))" 2>/dev/null || echo 5100)

echo "  Starting VR bridge server..."
echo "  VR device browser: https://127.0.0.1:$PORT"
echo "  ROS2 topics: /vr/head_pose, /vr/left_pose, /vr/right_pose,"
echo "               /vr/left_buttons, /vr/right_buttons, /vr_raw_data"
echo ""

exec "$PYTHON" vr_bridge_server.py "$@"

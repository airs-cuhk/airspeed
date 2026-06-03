#!/usr/bin/env bash
# Camera Stream ROS2 Publisher — Launch Script
#
# Discovers RealSense cameras and publishes Image + CameraInfo at native rate.
# Uses python3 from PATH — activate your env (conda/venv) before running.
#
# Usage:
#   bash launch/start.sh
#   bash launch/start.sh --config-dir /path/to/config

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT"

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

set -u

echo "============================================"
echo "  Camera Stream ROS2 Publisher"
echo "============================================"
echo ""
echo "  Rate: native (no artificial cap)"
echo "  Streams: per config/camera.yaml"
echo ""

exec python3 camera_publisher.py "$@"

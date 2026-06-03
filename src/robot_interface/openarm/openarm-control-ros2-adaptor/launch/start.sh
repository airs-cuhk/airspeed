#!/usr/bin/env bash
# OpenArm Control ROS2 Adaptor — Launch Script
#
# Runs the arm controller in the foreground. The controller handles:
#   - Calibration, homing, gripper test
#   - WebSocket streaming
#   - Starting/stopping the ROS2 arm_state_publisher
#   - Graceful shutdown (home return → disable torque → disconnect)
#
# Uses python3 from PATH — activate your env (conda/venv) before running.
#
# Usage:
#   bash launch/start.sh
#   bash launch/start.sh --ws-uri ws://192.168.1.100:5200/ws/arm

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT"

# Source ROS2 — must happen before set -u
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

set -u

exec python3 arm_controller.py "$@"

#!/usr/bin/env bash
# Start the canonical ROH hand driver node.
# Prereqs: ROS2 sourced; can2/can3 up at 1 Mbps (see README "CAN bus setup").
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADAPTOR_DIR="$(dirname "$SCRIPT_DIR")"

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

CONFIG="${1:-$ADAPTOR_DIR/config/roh_hand.yaml}"
# On 115 the system python3 lacks python-can — point PYTHON at a venv that
# has it (e.g. PYTHON=/home/intern/airspeed-canonical/.venv/bin/python).
PYTHON="${PYTHON:-python3}"
echo "=== ROH Hand Driver (canonical) ==="
echo "  Config: $CONFIG"
echo "  Python: $PYTHON"
echo "  Topics: /roh/<side>/joint_state (pub), /roh/<side>/joint_command (sub)"
echo ""
exec "$PYTHON" "$ADAPTOR_DIR/roh_hand_node.py" --config "$CONFIG"

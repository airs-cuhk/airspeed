#!/usr/bin/env bash
# Start the canonical VR→ROH hand controller (Joy in, JointState commands out).
# Prereqs: vr_bridge running (Joy topics); roh_hand_node running.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADAPTOR_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS2 BEFORE set -u — its setup scripts reference unset variables.
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
set -u

CONFIG="${1:-$ADAPTOR_DIR/config/roh_hand.yaml}"
PYTHON="${PYTHON:-python3}"
echo "=== ROH VR→Hand Controller (canonical) ==="
echo "  Config: $CONFIG"
echo "  Python: $PYTHON"
echo "  In:  /vr/<side>_buttons (Joy)"
echo "  Out: /roh/<side>/joint_command (JointState, radians)"
echo ""
exec "$PYTHON" "$ADAPTOR_DIR/roh_vr_controller_node.py" --config "$CONFIG"

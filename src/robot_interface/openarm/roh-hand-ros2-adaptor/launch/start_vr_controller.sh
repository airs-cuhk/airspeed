#!/usr/bin/env bash
# Start the canonical VR→ROH hand controller (Joy in, JointState commands out).
# Prereqs: ROS2 sourced; vr_bridge running (Joy topics); roh_hand_node running.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADAPTOR_DIR="$(dirname "$SCRIPT_DIR")"

if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

CONFIG="${1:-$ADAPTOR_DIR/config/roh_hand.yaml}"
echo "=== ROH VR→Hand Controller (canonical) ==="
echo "  Config: $CONFIG"
echo "  In:  /vr/<side>_buttons (Joy)"
echo "  Out: /roh/<side>/joint_command (JointState, radians)"
echo ""
exec python3 "$ADAPTOR_DIR/roh_vr_controller_node.py" --config "$CONFIG"

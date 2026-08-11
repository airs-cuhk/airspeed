#!/usr/bin/env bash
# Start the full canonical ROH hand stack with ONE command:
#   1. roh_hand_node          — CAN driver, /roh/<side>/joint_state pub
#   2. roh_vr_controller_node — VR→hand, /roh/<side>/joint_command pub
#
# Prereqs: can2/can3 up at 1 Mbps (sudo launch/setup_can.sh), vr_bridge
# running (Joy topics). Ctrl+C stops both nodes.
#
# Run WITHOUT sudo. As root the node works on CAN, but FastDDS shared-memory
# transport cannot exchange data with user-space processes (VR bridge,
# collector) — commands and states silently stop flowing (debugged
# 2026-08-11). SocketCAN itself needs no root.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ADAPTOR_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "$ADAPTOR_DIR/../../../.." && pwd)"
LOG_DIR="$ADAPTOR_DIR/logs"
mkdir -p "$LOG_DIR"

if [ "$EUID" -eq 0 ]; then
    echo "ERROR: do not run with sudo — root-owned ROS2 nodes cannot exchange" >&2
    echo "DDS data with the user-space VR bridge/collector (shm transport)." >&2
    echo "CAN access works unprivileged; just run: $0" >&2
    exit 1
fi

# Source ROS2 BEFORE set -u — its setup scripts reference unset variables.
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
set -u

CONFIG="${1:-$ADAPTOR_DIR/config/roh_hand.yaml}"
if [ -z "${PYTHON:-}" ] && [ -x "$REPO_ROOT/.venv/bin/python" ]; then
    PYTHON="$REPO_ROOT/.venv/bin/python"
fi
PYTHON="${PYTHON:-python3}"

echo "=== ROH Hand Stack (canonical) ==="
echo "  Config: $CONFIG"
echo "  Python: $PYTHON"
echo "  Driver log: $LOG_DIR/roh_hand_node.log"
echo ""

DRIVER_PID=""
VR_PID=""
cleanup() {
    trap - INT TERM EXIT
    echo ""
    echo "Stopping ROH hand stack..."
    for pid in $DRIVER_PID $VR_PID; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null
    echo "Done."
    exit 0
}
trap cleanup INT TERM EXIT

# 1. Driver first — it owns the CAN channels and subscribes joint_command.
"$PYTHON" "$ADAPTOR_DIR/roh_hand_node.py" --config "$CONFIG" \
    >"$LOG_DIR/roh_hand_node.log" 2>&1 &
DRIVER_PID=$!
sleep 3
if ! kill -0 "$DRIVER_PID" 2>/dev/null; then
    echo "ERROR: roh_hand_node died at startup — last log lines:" >&2
    tail -20 "$LOG_DIR/roh_hand_node.log" >&2
    exit 1
fi
echo "  driver running (pid $DRIVER_PID)"

# 2. VR→hand controller in the foreground of this terminal (background from
#    bash's view so `wait` stays trap-interruptible — a plain foreground call
#    would defer the TERM trap until the node exits and orphan the driver).
"$PYTHON" "$ADAPTOR_DIR/roh_vr_controller_node.py" --config "$CONFIG" &
VR_PID=$!
echo "  vr controller running (pid $VR_PID)"
wait "$VR_PID"

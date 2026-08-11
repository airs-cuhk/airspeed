#!/usr/bin/env bash
# Start the canonical ROH hand driver node.
# Prereqs: can2/can3 up at 1 Mbps (see README "CAN bus setup").
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
# Default to the repo-root venv: it has python-can and the SDK deps, and an
# absolute path survives `sudo` (which drops the caller's activated venv).
# Override with PYTHON=/path/to/python if needed.
REPO_ROOT="$(cd "$ADAPTOR_DIR/../../../.." && pwd)"
if [ -z "${PYTHON:-}" ] && [ -x "$REPO_ROOT/.venv/bin/python" ]; then
    PYTHON="$REPO_ROOT/.venv/bin/python"
fi
PYTHON="${PYTHON:-python3}"
echo "=== ROH Hand Driver (canonical) ==="
echo "  Config: $CONFIG"
echo "  Python: $PYTHON"
echo "  Topics: /roh/<side>/joint_state (pub), /roh/<side>/joint_command (sub)"
echo ""
exec "$PYTHON" "$ADAPTOR_DIR/roh_hand_node.py" --config "$CONFIG"

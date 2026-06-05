#!/usr/bin/env bash
# Data Collection Service — Config-driven launcher.
#
# Reads global_config.yaml for session config, output dir, and UI settings,
# then starts the collector via ros2 launch.
#
# Usage:
#   bash run_global_config.sh

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Read config
SESSION=$(python3 -c "import yaml; print(yaml.safe_load(open('global_config.yaml'))['session_config'])")
OUTPUT=$(python3 -c "import yaml; print(yaml.safe_load(open('global_config.yaml'))['output_dir'])")
UI_HOST=$(python3 -c "import yaml; print(yaml.safe_load(open('global_config.yaml'))['operator_ui_host'])")
UI_PORT=$(python3 -c "import yaml; print(yaml.safe_load(open('global_config.yaml'))['operator_ui_port'])")

# Source ROS2 before set -u
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
set -u

echo "=== Data Collection Service ==="
echo "  Session:  $SESSION"
echo "  Output:   $OUTPUT"
echo "  UI:       http://${UI_HOST}:${UI_PORT}"
echo ""

export DATA_COLLECTION_SERVICE_ROOT="$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

python3 -m core.runtime.ros2_collection_node \
    --session-config "$SESSION" \
    --output-dir "$OUTPUT" \
    --operator-ui-host "$UI_HOST" \
    --operator-ui-port "$UI_PORT" \
    "$@"

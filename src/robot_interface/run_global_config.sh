#!/usr/bin/env bash
# Robot Interface — Config-driven launcher.
#
# Reads global_config.yaml to determine which adaptor to start.
# Uses python3 from PATH — activate your env (conda/venv) before running.
#
# Usage:
#   bash run_global_config.sh                          # uses adaptor from global_config.yaml
#   bash run_global_config.sh --ws-uri ws://host:5200  # passed through to adaptor

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ADAPTOR=$(python3 -c "import yaml; print(yaml.safe_load(open('global_config.yaml'))['adaptor'])")
LAUNCH_SCRIPT="$SCRIPT_DIR/$ADAPTOR/launch/start.sh"
if [ ! -f "$LAUNCH_SCRIPT" ]; then
    echo "ERROR: launch script not found: $LAUNCH_SCRIPT"
    exit 1
fi

echo "=== Robot Interface ==="
echo "  Adaptor: $ADAPTOR"
echo "  Python:  $(which python3)"
echo "  Launch:  $LAUNCH_SCRIPT"
echo ""

exec bash "$LAUNCH_SCRIPT" "$@"

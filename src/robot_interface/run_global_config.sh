#!/usr/bin/env bash
# Robot Interface — Config-driven launcher.
#
# Reads global_config.yaml to determine which adaptor to start, then runs its
# launch/start.sh.  Passes all CLI arguments through to the adaptor script.
#
# Usage:
#   bash run_global_config.sh                          # uses adaptor from global_config.yaml
#   bash run_global_config.sh --ws-uri ws://host:5200  # passed through to adaptor
#
# To switch adaptors, edit global_config.yaml — no need to change this script.

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PYTHON=""
for py in /usr/bin/python3.10 python3.10 python3; do
    if command -v "$py" &>/dev/null && "$py" -c "import yaml" 2>/dev/null; then
        PYTHON="$py"; break
    fi
done
if [ -z "$PYTHON" ]; then
    echo "ERROR: no Python with PyYAML found. Install: pip install pyyaml"
    exit 1
fi
ADAPTOR=$("$PYTHON" -c "import yaml; print(yaml.safe_load(open('global_config.yaml'))['adaptor'])")
if [ -z "$ADAPTOR" ]; then
    echo "ERROR: could not read 'adaptor' from global_config.yaml"
    exit 1
fi
CFG_PYTHON=$("$PYTHON" -c "import yaml; print(yaml.safe_load(open('global_config.yaml')).get('python',''))" 2>/dev/null)
if [ -n "$CFG_PYTHON" ]; then
    export PYTHON_BIN="$CFG_PYTHON"
fi

LAUNCH_SCRIPT="$SCRIPT_DIR/$ADAPTOR/launch/start.sh"
if [ ! -f "$LAUNCH_SCRIPT" ]; then
    echo "ERROR: launch script not found: $LAUNCH_SCRIPT"
    exit 1
fi

echo "=== Robot Interface ==="
echo "  Adaptor: $ADAPTOR"
echo "  Python:  ${PYTHON_BIN:-auto-detect}"
echo "  Launch:  $LAUNCH_SCRIPT"
echo ""

exec bash "$LAUNCH_SCRIPT" "$@"

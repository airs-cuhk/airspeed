#!/usr/bin/env bash
# Robot Interface — Config-driven launcher.
#
# Reads global_config.yaml to determine which adaptors to start, then runs
# their launch/start.sh scripts concurrently. Ctrl+C stops all adaptors.
#
# Usage:
#   bash run_global_config.sh
#   bash run_global_config.sh --ws-uri ws://host:5200  # passed through to adaptors

set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Read adaptors list (with fallback to legacy singular 'adaptor' key)
ADAPTORS=$(python3 -c "
import yaml
cfg = yaml.safe_load(open('global_config.yaml'))
items = cfg.get('adaptors') or [cfg.get('adaptor')]
print(' '.join(items))
")
if [ -z "$ADAPTORS" ]; then
    echo "ERROR: no adaptors configured in global_config.yaml"
    exit 1
fi

# Validate all exist first
for a in $ADAPTORS; do
    SCRIPT="$SCRIPT_DIR/$a/launch/start.sh"
    if [ ! -f "$SCRIPT" ]; then
        echo "ERROR: launch script not found: $SCRIPT"
        exit 1
    fi
done

echo "=== Robot Interface ==="
echo "  Python:  $(which python3)"
for a in $ADAPTORS; do echo "  Adaptor: $a"; done
echo ""

# Start all adaptors in background; track PIDs
PIDS=""
for a in $ADAPTORS; do
    bash "$SCRIPT_DIR/$a/launch/start.sh" "$@" &
    PIDS="$PIDS $!"
done

# Trap Ctrl+C — kill all children, then exit
cleanup() {
    echo ""
    echo "Stopping all adaptors..."
    for pid in $PIDS; do kill "$pid" 2>/dev/null || true; done
    wait 2>/dev/null
    echo "Done."
    exit 0
}
trap cleanup INT TERM

wait

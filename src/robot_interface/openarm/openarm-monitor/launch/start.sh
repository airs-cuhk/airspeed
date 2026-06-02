#!/usr/bin/env bash
# OpenArm 3D Monitor — Launch Script
#
# Standalone ROS2 subscriber + WebSocket server for the 3D monitoring UI.
# Completely decoupled from the IK solver.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT"

CONFIG_FILE="$ROOT/config/monitor.yaml"

yaml_get() {
    local _file="$1" _key="$2"
    if [[ "$_key" == *.* ]]; then
        local _section="${_key%%.*}" _field="${_key##*.}"
        awk -v section="^${_section}:" -v field="^[[:space:]]*${_field}:" '
            $0 ~ section { in_section=1; next }
            in_section && $0 ~ field { val=$0; sub(/^[^:]*:[[:space:]]*/, "", val); sub(/[[:space:]]*#.*$/, "", val); gsub(/^"|"$/, "", val); print val; exit }
            /^[a-z]/ { in_section=0 }
        ' "$_file"
    else
        awk -v key="^[[:space:]]*${_key}:" '
            $0 ~ key { val=$0; sub(/^[^:]*:[[:space:]]*/, "", val); sub(/[[:space:]]*#.*$/, "", val); gsub(/^"|"$/, "", val); print val; exit }
        ' "$_file"
    fi
}

setup_python_env() {
    local _type
    _type=$(yaml_get "$CONFIG_FILE" "python_env.type")
    case "${_type:-system}" in
        conda)
            local _ch _ce
            _ch=$(yaml_get "$CONFIG_FILE" "python_env.conda_home")
            _ch="${_ch/#\~/$HOME}"
            _ce=$(yaml_get "$CONFIG_FILE" "python_env.conda_env")
            [ -z "$_ch" ] || [ -z "$_ce" ] && { echo "ERROR: conda_home and conda_env required"; exit 1; }
            source "$_ch/etc/profile.d/conda.sh"
            conda activate "$_ce"
            PYTHON_BIN="$(which python3.10)"
            export PYTHON_BIN
            ;;
        venv)
            local _vp
            _vp=$(yaml_get "$CONFIG_FILE" "python_env.venv_path")
            _vp="${_vp/#\~/$HOME}"
            [ -z "$_vp" ] || [ ! -f "$_vp/bin/activate" ] && { echo "ERROR: venv_path required"; exit 1; }
            source "$_vp/bin/activate"
            PYTHON_BIN="$(which python3.10)"
            export PYTHON_BIN
            ;;
        system) ;;
        *) echo "ERROR: unknown python_env.type '$_type'"; exit 1 ;;
    esac
}

check_python() {
    if [ -n "${PYTHON_BIN:-}" ]; then PYTHON="$PYTHON_BIN"; return 0; fi
    for py in python3.10 python3 /usr/bin/python3.10; do
        if command -v "$py" &>/dev/null; then PYTHON="$py"; return 0; fi
    done
    echo "ERROR: Python 3.10+ not found."; exit 1
}

# Source ROS2 before set -u
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

setup_python_env
check_python
set -u

echo "============================================"
echo "  OpenArm 3D Monitor"
echo "============================================"
exec "$PYTHON" monitor_server.py "$@"

#!/usr/bin/env bash
# OpenArm Control ROS2 Adaptor — Launch Script
#
# Runs the arm controller in the foreground. The controller handles:
#   - Calibration, homing, gripper test
#   - WebSocket streaming
#   - Starting/stopping the ROS2 arm_state_publisher
#   - Graceful shutdown (home return → disable torque → disconnect)
#
# Usage:
#   bash launch/start.sh
#   bash launch/start.sh --ws-uri ws://192.168.1.100:5200/ws/arm

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
cd "$ROOT"

CONFIG_FILE="$ROOT/config/robot.yaml"

# ---------------------------------------------------------------------------
# YAML helper — read a top-level scalar from a simple YAML key: value line.
# Handles nested keys like "python_env.conda_home" (max 1 level deep).
# ---------------------------------------------------------------------------
yaml_get() {
    local _file="$1"
    local _key="$2"

    if [[ "$_key" == *.* ]]; then
        local _section="${_key%%.*}"
        local _field="${_key##*.}"
        awk -v section="^${_section}:" -v field="^[[:space:]]*${_field}:" '
            $0 ~ section { in_section=1; next }
            in_section && $0 ~ field {
                val=$0
                sub(/^[^:]*:[[:space:]]*/, "", val)
                sub(/[[:space:]]*#.*$/, "", val)
                gsub(/^"|"$/, "", val)
                print val
                exit
            }
            /^[a-z]/ { in_section=0 }
        ' "$_file"
    else
        awk -v key="^[[:space:]]*${_key}:" '
            $0 ~ key {
                val=$0
                sub(/^[^:]*:[[:space:]]*/, "", val)
                sub(/[[:space:]]*#.*$/, "", val)
                gsub(/^"|"$/, "", val)
                print val
                exit
            }
        ' "$_file"
    fi
}

# ---------------------------------------------------------------------------
# Setup Python environment from config/robot.yaml
# ---------------------------------------------------------------------------
setup_python_env() {
    local _type
    _type=$(yaml_get "$CONFIG_FILE" "python_env.type")

    case "${_type:-system}" in
        conda)
            local _conda_home _conda_env
            _conda_home=$(yaml_get "$CONFIG_FILE" "python_env.conda_home")
            _conda_home="${_conda_home/#\~/$HOME}"
            _conda_env=$(yaml_get "$CONFIG_FILE" "python_env.conda_env")

            if [ -z "$_conda_home" ] || [ -z "$_conda_env" ]; then
                echo "ERROR: python_env.type=conda requires conda_home and conda_env in $CONFIG_FILE"
                exit 1
            fi
            if [ ! -f "$_conda_home/etc/profile.d/conda.sh" ]; then
                echo "ERROR: conda.sh not found at $_conda_home/etc/profile.d/conda.sh"
                exit 1
            fi
            source "$_conda_home/etc/profile.d/conda.sh"
            conda activate "$_conda_env"
            PYTHON_BIN="$(which python3.10)"
            export PYTHON_BIN
            ;;

        venv)
            local _venv_path
            _venv_path=$(yaml_get "$CONFIG_FILE" "python_env.venv_path")
            _venv_path="${_venv_path/#\~/$HOME}"
            if [ -z "$_venv_path" ] || [ ! -f "$_venv_path/bin/activate" ]; then
                echo "ERROR: python_env.type=venv requires venv_path with a valid venv in $CONFIG_FILE"
                exit 1
            fi
            source "$_venv_path/bin/activate"
            PYTHON_BIN="$(which python3.10)"
            export PYTHON_BIN
            ;;

        system)
            ;;  # rely on check_python fallback below

        *)
            echo "ERROR: unknown python_env.type '$_type' in $CONFIG_FILE (expected: conda | venv | system)"
            exit 1
            ;;
    esac
}

# ---------------------------------------------------------------------------
# Find Python (before set -u — conda scripts reference unbound vars)
# ---------------------------------------------------------------------------

check_python() {
    if [ -n "${PYTHON_BIN:-}" ]; then
        PYTHON="$PYTHON_BIN"
        return 0
    fi
    for py in python3.10 python3 /usr/bin/python3.10; do
        if command -v "$py" &>/dev/null; then
            PYTHON="$py"
            return 0
        fi
    done
    echo "ERROR: Python 3.10+ not found."
    exit 1
}

# Source ROS2 — must happen before set -u
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

setup_python_env
check_python

set -u

# ---------------------------------------------------------------------------
# Run the controller in the foreground — it handles everything
# ---------------------------------------------------------------------------
exec "$PYTHON" arm_controller.py "$@"

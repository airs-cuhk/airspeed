#!/usr/bin/env bash
# OpenArm IK ROS2 Adaptor — Launch Script
#
# Starts the IK validation server with JAX JIT solver at 50 Hz.
# Uses python3 from PATH — activate your env (conda/venv) before running.
#
# Usage:
#   bash launch/start.sh
#   bash launch/start.sh --seed-caches   # one-time cache setup, no network needed after

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source ROS2 if available (before set -u — ROS2 scripts reference unbound vars)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

set -u

# --- Cache seeding ---
if [[ "${1:-}" == "--seed-caches" ]]; then
    echo "[run_server] Cache seeding complete. Caches are ready, no network needed on next start."
    exit 0
fi

# JAX compilation cache: symlink bundle .cache → ~/.cache if not already set up
_JAX_CACHE_BUNDLE="${PROJECT_ROOT}/.cache/jax_compilation"
_JAX_CACHE_HOME="${HOME}/.cache/jax_compilation"
if [ -d "$_JAX_CACHE_BUNDLE" ] && [ ! -d "$_JAX_CACHE_HOME" ]; then
    mkdir -p "$(dirname "$_JAX_CACHE_HOME")"
    ln -sfn "$_JAX_CACHE_BUNDLE" "$_JAX_CACHE_HOME"
    echo "[run_server] Linked JAX cache: ${_JAX_CACHE_HOME} -> ${_JAX_CACHE_BUNDLE}"
fi
export JAX_COMPILATION_CACHE_DIR="${HOME}/.cache/jax_compilation"

# RAM URDF cache: symlink bundle .cache/ram → ~/.cache/ram if not already set up
_RAM_CACHE_BUNDLE="${PROJECT_ROOT}/.cache/ram"
_RAM_CACHE_HOME="${HOME}/.cache/ram"
if [ -d "$_RAM_CACHE_BUNDLE" ] && [ ! -d "$_RAM_CACHE_HOME" ]; then
    mkdir -p "$(dirname "$_RAM_CACHE_HOME")"
    ln -sfn "$_RAM_CACHE_BUNDLE" "$_RAM_CACHE_HOME"
    echo "[run_server] Linked RAM cache: ${_RAM_CACHE_HOME} -> ${_RAM_CACHE_BUNDLE}"
fi

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0
export JAX_PLATFORMS=cpu
export PYTHONPATH="${PROJECT_ROOT}/.pydeps:${PROJECT_ROOT}/openarm_ik_solver:${PYTHONPATH:-}"

echo "[run_server] Starting IK validation server..."
echo "[run_server] Project root: $PROJECT_ROOT"
echo "[run_server] Python: $(which python3)"
echo "[run_server] ROS2: ${ROS_DISTRO:-not sourced}"
echo "[run_server] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

cd "$PROJECT_ROOT"
exec python3 -m server.main "$@"

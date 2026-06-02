#!/usr/bin/env bash
# Start the IK validation server with correct Python version and paths.
# Bundle layout: everything is self-contained under PROJECT_ROOT.
#
# Usage:
#   bash launch/start.sh
#
# Cache seeding (run once on a fresh target machine to avoid network fetches):
#   bash launch/start.sh --seed-caches

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source ROS2 if available (must happen before set -u because ROS2 scripts reference unbound vars)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

set -u

# --- Cache seeding ---
SEED_CACHES=false
if [[ "${1:-}" == "--seed-caches" ]]; then
    SEED_CACHES=true
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

if $SEED_CACHES; then
    echo "[run_server] Cache seeding complete. Caches are ready, no network needed on next start."
    exit 0
fi

# ROS_DOMAIN_ID: set to match the existing ROS2 ecosystem on this machine.
# Default is 0 (unset). Override with: ROS_DOMAIN_ID=<N> bash launch/start.sh
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export ROS_LOCALHOST_ONLY=0

# Force CPU backend — lower solve latency than GPU for this workload
export JAX_PLATFORMS=cpu

# Set PYTHONPATH for self-contained bundle
export PYTHONPATH="${PROJECT_ROOT}/.pydeps:${PROJECT_ROOT}/openarm_ik_solver:${PYTHONPATH:-}"

# Use configured Python or auto-detect (deps compiled for 3.10)
if [ -n "${PYTHON_BIN:-}" ]; then
    PYTHON="$PYTHON_BIN"
else
    PYTHON="/usr/bin/python3.10"
fi
if ! command -v "$PYTHON" &> /dev/null; then
    echo "Error: Python not found at $PYTHON"
    echo "Set python: in global_config.yaml or install /usr/bin/python3.10"
    exit 1
fi

echo "[run_server] Starting IK validation server..."
echo "[run_server] Project root: $PROJECT_ROOT"
echo "[run_server] Python: $PYTHON"
echo "[run_server] PYTHONPATH: $PYTHONPATH"
echo "[run_server] ROS2: ${ROS_DISTRO:-not sourced}"
echo "[run_server] ROS_DOMAIN_ID=$ROS_DOMAIN_ID (all ROS2 nodes must share this domain)"

cd "$PROJECT_ROOT"
exec "$PYTHON" -m server.main "$@"

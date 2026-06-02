"""Application entry point. Starts aiohttp server with IK validation backend."""

from __future__ import annotations

import sys
import os
import argparse
from pathlib import Path

# Add .pydeps/ and openarm_ik_solver/ to path (both at bundle root)
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_PYDEPS = _PROJECT_ROOT / ".pydeps"
_IK_SOLVER = _PROJECT_ROOT / "openarm_ik_solver"

if _PYDEPS.exists():
    sys.path.insert(0, str(_PYDEPS))
else:
    raise FileNotFoundError(f"Python dependencies not found at {_PYDEPS}. "
                            f"Run bootstrap_deps.sh first or copy .pydeps/ into the bundle.")
if _IK_SOLVER.exists():
    sys.path.insert(0, str(_IK_SOLVER))

# Add ROS2 Humble Python packages if available (needed for rclpy, geometry_msgs)
_ROS2_PY = Path("/opt/ros/humble/lib/python3.10/site-packages")
_ROS2_LOCAL_PY = Path("/opt/ros/humble/local/lib/python3.10/dist-packages")
for _ros_path in (_ROS2_PY, _ROS2_LOCAL_PY):
    if _ros_path.exists() and str(_ros_path) not in sys.path:
        sys.path.append(str(_ros_path))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="OpenArm IK Validation Server",
    )
    parser.add_argument(
        "--solver-config",
        type=Path,
        default=Path("config/solver_smooth.yaml"),
        help=(
            "Path to solver YAML config (default: config/solver_smooth.yaml). "
            "Example: --solver-config config/solver_stable.yaml"
        ),
    )
    return parser.parse_args()


def main() -> None:
    from aiohttp import web
    from server.config_loader import load_config
    from server.ws_handler import create_ws_app

    args = _parse_args()
    solver_path = args.solver_config
    if solver_path is not None and not solver_path.is_absolute():
        solver_path = _PROJECT_ROOT / solver_path

    config = load_config(_PROJECT_ROOT / "config", solver_config_path=solver_path)
    app = create_ws_app(config)

    host = config.server.host
    port = config.server.port
    print(f"3D monitoring UI: http://localhost:{port}/web/index.html")
    web.run_app(app, host=host, port=port)


if __name__ == "__main__":
    main()

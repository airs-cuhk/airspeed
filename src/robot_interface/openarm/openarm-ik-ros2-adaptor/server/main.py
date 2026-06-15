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
    _msg = (
        "Dependencies not bundled. Install them with pip:\n"
        "  pip install jax[cpu] jaxlie jaxls pyroki yourdfpy aiohttp pyyaml \\\n"
        "              numpy scipy trimesh lxml pydantic tyro rich\n"
        "Or for a self-contained bundle:\n"
        "  pip install --target .pydeps jax[cpu] jaxlie jaxls pyroki yourdfpy aiohttp pyyaml \\\n"
        "                      numpy scipy trimesh lxml pydantic tyro rich\n"
        f"Then set PYTHONPATH={_PYDEPS}:{_IK_SOLVER}"
    )
    print(_msg)
    sys.exit(1)
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


def _start_webui_static_server(webui_root: Path, port: int = 8080) -> None:
    """Serve webui-monitor static files on a dedicated port (daemon thread)."""
    import time
    import threading
    import webbrowser
    from http.server import SimpleHTTPRequestHandler, HTTPServer

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(webui_root), **kwargs)

    def _serve():
        server = HTTPServer(("0.0.0.0", port), Handler)
        print(f"[startup] 3D monitoring UI: http://localhost:{port}/web_pages/index.html")
        server.serve_forever()

    threading.Thread(target=_serve, daemon=True).start()
    time.sleep(0.3)
    webbrowser.open(f"http://localhost:{port}/web_pages/index.html")


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

    # Start dedicated static file server for the 3D monitoring UI on port 8080
    _start_webui_static_server(_PROJECT_ROOT / "webui-monitor")

    host = config.server.host
    port = config.server.port
    print(f"[startup] IK service: http://localhost:{port}")
    web.run_app(app, host=host, port=port)


if __name__ == "__main__":
    main()

"""WebSocket handler and aiohttp app factory."""

from __future__ import annotations

import json
from typing import Any

from aiohttp import web, WSCloseCode

from server.config_loader import AppConfig
from server.ik_service import IKService
from server.target_buffer import TargetBuffer


async def websocket_handler(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    app = request.app
    app["websockets"].add(ws)

    buffer: TargetBuffer = app["target_buffer"]

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                data = json.loads(msg.data)
                msg_type = data.get("type")

                if msg_type == "target_update":
                    buffer.write(data)

                elif msg_type == "vr_connect":
                    vr_sub = app.get("vr_subscriber")
                    if vr_sub is not None:
                        success = vr_sub.restart()
                        await ws.send_str(json.dumps({
                            "type": "vr_connect_response",
                            "success": success,
                            "ros2_installed": vr_sub.ros2_installed,
                            "running": vr_sub.is_running,
                        }))
                    else:
                        await ws.send_str(json.dumps({
                            "type": "vr_connect_response",
                            "success": False,
                            "error": "VR subscriber not initialized",
                        }))

                elif msg_type == "vr_disconnect":
                    vr_sub = app.get("vr_subscriber")
                    vr_norm = app.get("vr_normalizer")
                    if vr_sub is not None:
                        vr_sub.stop()
                    if vr_norm is not None:
                        vr_norm._reset()
                    await ws.send_str(json.dumps({
                        "type": "vr_disconnect_response",
                        "success": True,
                    }))

                elif msg_type == "reset_home":
                    app["reset_home_requested"] = True
                    await ws.send_str(json.dumps({
                        "type": "reset_home_response",
                        "success": True,
                    }))

            elif msg.type == web.WSMsgType.ERROR:
                print(f"WebSocket error: {ws.exception()}")

    finally:
        app["websockets"].discard(ws)

    return ws


async def arm_ws_handler(request: web.Request) -> web.WebSocketResponse:
    """WebSocket endpoint for streaming arm joint commands (18 floats at ~50Hz)."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    app = request.app
    app["arm_websockets"].add(ws)

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.ERROR:
                print(f"Arm WebSocket error: {ws.exception()}")
    finally:
        app["arm_websockets"].discard(ws)

    return ws


async def audit_handler(request: web.Request) -> web.Response:
    """HTTP endpoint to export audit log."""
    fmt = request.query.get("format", "json")
    ik_service: IKService = request.app["ik_service"]

    if fmt == "csv":
        csv_data = ik_service.audit_log.to_csv()
        return web.Response(
            text=csv_data,
            content_type="text/csv",
            headers={"Content-Disposition": "attachment; filename=audit_log.csv"},
        )

    entries = ik_service.audit_log.get_entries()
    return web.json_response(entries)


async def on_startup(app: web.Application) -> None:
    app["websockets"] = set()
    app["arm_websockets"] = set()
    app["solve_index"] = 0
    app["reset_home_requested"] = False

    config: AppConfig = app["config"]

    # Create target buffer (legacy drag — still supported)
    app["target_buffer"] = TargetBuffer()

    # Initialize rclpy once in the main thread before any ROS2 daemon threads
    # start. This prevents race conditions between subscriber and publisher.
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
            app["_rclpy_initialized"] = True
    except ImportError:
        app["_rclpy_initialized"] = False

    # Create VR subscriber and normalizer
    from server.vr_subscriber import VRSubscriber
    from server.vr_normalizer import VRNormalizer

    vr_subscriber = VRSubscriber(config.vr)
    vr_normalizer = VRNormalizer(config.vr)
    vr_subscriber.start()  # Graceful fallback if ROS2 not installed
    app["vr_subscriber"] = vr_subscriber
    app["vr_normalizer"] = vr_normalizer

    # Create ROS2 joint command publisher (graceful fallback if ROS2 not installed)
    from server.ros2_publisher import ArmCommandPublisher

    left_joint_names = [
        "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
        "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
        "openarm_left_joint7", "openarm_left_finger_joint1",
    ]
    right_joint_names = [
        "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
        "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
        "openarm_right_joint7", "openarm_right_finger_joint1",
    ]

    ros2_publisher: ArmCommandPublisher | None = None
    if config.vr.publisher is not None and config.vr.publisher.enabled:
        ros2_publisher = ArmCommandPublisher(
            left_joint_names=left_joint_names,
            right_joint_names=right_joint_names,
        )
        ros2_publisher.start()
    app["ros2_publisher"] = ros2_publisher

    # Load IK solver (this triggers JIT warmup; cached runs are near-instant)
    import os
    cache_dir = os.environ.get("JAX_COMPILATION_CACHE_DIR", "~/.cache/jax_compilation")
    print(f"[startup] Loading IK solver (JIT warmup, cache_dir={cache_dir})...")
    ik_service = IKService(config)
    app["ik_service"] = ik_service
    print(f"[startup] IK solver ready (warmup_complete={ik_service.solver.warmup_complete})")

    # Supply FK home poses to normalizer for calibration targets
    vr_normalizer.set_home_ee(ik_service.home_fk)

    # Start solver loop as background task
    import asyncio
    from server.solver_loop import solver_loop

    app["solver_loop_task"] = asyncio.get_event_loop().create_task(
        solver_loop(
            app=app,
            ik_service=app["ik_service"],
            buffer=app["target_buffer"],
            config=config,
            vr_subscriber=vr_subscriber,
            vr_normalizer=vr_normalizer,
            ros2_publisher=ros2_publisher,
        )
    )


async def on_shutdown(app: web.Application) -> None:
    # Cancel solver loop
    task = app.get("solver_loop_task")
    if task is not None:
        task.cancel()

    # Stop VR subscriber
    vr_sub = app.get("vr_subscriber")
    if vr_sub is not None:
        vr_sub.stop()

    # Stop ROS2 publisher
    ros2_pub = app.get("ros2_publisher")
    if ros2_pub is not None:
        ros2_pub.stop()

    # Shutdown rclpy once after all ROS2 threads have stopped
    if app.get("_rclpy_initialized"):
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass

    # Close all WebSocket connections
    for ws in set(app["websockets"]):
        await ws.close(code=WSCloseCode.GOING_AWAY, message=b"Server shutdown")
    for ws in set(app["arm_websockets"]):
        await ws.close(code=WSCloseCode.GOING_AWAY, message=b"Server shutdown")


def create_ws_app(config: AppConfig) -> web.Application:
    app = web.Application()
    app["config"] = config

    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    # Root — serve web UI
    from pathlib import Path as _Path
    _web_root = _Path(__file__).resolve().parent.parent / "web"
    async def _index(_): return web.FileResponse(_web_root / "index.html")
    app.router.add_get("/", _index)

    # WebSocket endpoint
    app.router.add_get("/ws", websocket_handler)

    # Arm command streaming WebSocket endpoint
    app.router.add_get("/ws/arm", arm_ws_handler)

    # Audit export endpoint
    app.router.add_get("/api/audit", audit_handler)

    # Static file routes from config
    for route_config in config.server.static_routes:
        url_prefix = route_config["url_prefix"]
        fs_path = route_config["filesystem_path"]

        from pathlib import Path
        project_root = Path(__file__).resolve().parent.parent
        resolved = (project_root / fs_path).resolve()

        if resolved.exists():
            app.router.add_static(url_prefix, str(resolved))
        else:
            print(f"[warning] Static path not found: {resolved}")

    return app

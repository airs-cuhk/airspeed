"""WebSocket handler and aiohttp app factory — arm command streaming only."""

from __future__ import annotations

from aiohttp import web, WSCloseCode

from server.config_loader import AppConfig
from server.ik_service import IKService


async def arm_ws_handler(request: web.Request) -> web.WebSocketResponse:
    """WebSocket endpoint for streaming arm joint commands."""
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


async def on_startup(app: web.Application) -> None:
    app["arm_websockets"] = set()
    app["solve_index"] = 0
    app["reset_home_requested"] = False

    config: AppConfig = app["config"]

    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
            app["_rclpy_initialized"] = True
    except ImportError:
        app["_rclpy_initialized"] = False

    # VR subscriber and normalizer
    from server.vr_subscriber import VRSubscriber
    from server.vr_normalizer import VRNormalizer

    vr_subscriber = VRSubscriber(config.vr)
    vr_normalizer = VRNormalizer(config.vr)
    vr_subscriber.start()
    app["vr_subscriber"] = vr_subscriber
    app["vr_normalizer"] = vr_normalizer

    # ROS2 joint command publisher
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

    # IK solver (JIT warmup)
    import os
    cache_dir = os.environ.get("JAX_COMPILATION_CACHE_DIR", "~/.cache/jax_compilation")
    print(f"[startup] Loading IK solver (JIT warmup, cache_dir={cache_dir})...")
    ik_service = IKService(config)
    app["ik_service"] = ik_service
    print(f"[startup] IK solver ready (warmup_complete={ik_service.solver.warmup_complete})")

    vr_normalizer.set_home_ee(ik_service.home_fk)

    # Start solver loop
    import asyncio
    from server.solver_loop import solver_loop

    app["solver_loop_task"] = asyncio.get_event_loop().create_task(
        solver_loop(
            app=app,
            ik_service=app["ik_service"],
            config=config,
            vr_subscriber=vr_subscriber,
            vr_normalizer=vr_normalizer,
            ros2_publisher=ros2_publisher,
        )
    )


async def on_shutdown(app: web.Application) -> None:
    task = app.get("solver_loop_task")
    if task is not None:
        task.cancel()

    vr_sub = app.get("vr_subscriber")
    if vr_sub is not None:
        vr_sub.stop()

    ros2_pub = app.get("ros2_publisher")
    if ros2_pub is not None:
        ros2_pub.stop()

    if app.get("_rclpy_initialized"):
        try:
            import rclpy
            rclpy.shutdown()
        except Exception:
            pass

    for ws in set(app["arm_websockets"]):
        await ws.close(code=WSCloseCode.GOING_AWAY, message=b"Server shutdown")


def create_ws_app(config: AppConfig) -> web.Application:
    app = web.Application()
    app["config"] = config

    app.on_startup.append(on_startup)
    app.on_shutdown.append(on_shutdown)

    # Arm command streaming WebSocket
    app.router.add_get("/ws/arm", arm_ws_handler)

    return app

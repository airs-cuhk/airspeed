# OpenArm IK ROS2 Adaptor

Self-contained deployable IK pipeline for the OpenArm bimanual robot. Subscribes to
VR controller poses via ROS2, solves inverse kinematics at 50 Hz, and publishes joint
commands as standard `JointState` + `PoseStamped` matching the
[AIRSPEED robot interface convention](../../README.md).

Includes a browser-based 3D monitoring UI in `webui-monitor/` — a self-contained static
web app that can be copied anywhere and served independently. Zero network needed
after first cache seeding.

## Architecture

```
VR Bridge (/vr/*) ──ROS2──> vr_subscriber ──> vr_normalizer ──> IK solver (JAX, 50 Hz)
                                                                       │
                                                    ┌──────────────────┼──────────────────┐
                                                    ▼                                     ▼
                                           ros2_publisher                          ws_handler
                                          (JointState +                            (WebSocket)
                                           PoseStamped)                                │
                                                    │                                     ▼
                                           ROS2 topics                   webui-monitor/ (self-contained)
                                           /arm/left/*                   http://<host>:5200
                                           /arm/right/*                  WS at /ws (50 Hz snapshots)
```

## Published ROS2 Topics

| Topic | Message Type | Content |
|-------|-------------|---------|
| `/arm/left/joint_commands` | `sensor_msgs/JointState` | Left arm: 7 joints + gripper (radians) |
| `/arm/right/joint_commands` | `sensor_msgs/JointState` | Right arm: 7 joints + gripper (radians) |
| `/arm/left/target_pose` | `geometry_msgs/PoseStamped` | Left end-effector IK target |
| `/arm/right/target_pose` | `geometry_msgs/PoseStamped` | Right end-effector IK target |

Each `JointState` includes `header.stamp`, `header.frame_id`, `name[]` (URDF joint
names), and `position[]` (radians). These topics follow the
[robot interface data convention](../../README.md) — the data collection service can
subscribe to any subset declared in its session YAML.

## Subscribed ROS2 Topics (from VR Bridge)

| Topic | Message Type | Source |
|-------|-------------|--------|
| `/vr/head_pose` | `geometry_msgs/PoseStamped` | [VR bridge adaptor](../../../teleoperation_interface/vr-standard-ros2-bridge-adaptor/) |
| `/vr/left_pose` | `geometry_msgs/PoseStamped` | VR bridge |
| `/vr/right_pose` | `geometry_msgs/PoseStamped` | VR bridge |
| `/vr/left_buttons` | `std_msgs/Float32MultiArray` | VR bridge (trigger = gripper) |
| `/vr/right_buttons` | `std_msgs/Float32MultiArray` | VR bridge (trigger = gripper) |

## Prerequisites

| Requirement | Check | Mandatory |
|-----------|-------|-----------|
| Python 3.10 | `python3 --version` | Yes |
| ROS2 Humble | `source /opt/ros/humble/setup.bash` | No — degrades gracefully |
| x86_64 Linux | `uname -m` | Yes (.pydeps binaries are x86_64) |

## Quick Start

```bash
cd openarm-ik-ros2-adaptor

# 1. Seed caches — one-time setup, skips all network fetches
bash launch/start.sh --seed-caches

# 2. Start the server
bash launch/start.sh
```

The server listens on `http://0.0.0.0:5200`. Open it in a browser to see the 3D
monitoring UI with real-time robot visualization.

Requires the [VR bridge adaptor](../../../teleoperation_interface/vr-standard-ros2-bridge-adaptor/)
to be running — the adaptor subscribes to its ROS2 topics.

## How It Works

1. **VR subscriber** (`server/vr_subscriber.py`) — subscribes to VR pose and button
   topics via ROS2, stores latest data in a thread-safe store. Gracefully degrades
   when ROS2 is not installed.

2. **VR normalizer** (`server/vr_normalizer.py`) — calibrates the VR coordinate frame
   to the robot base frame. Operator presses B-button to pin origin, then moves
   controllers to define the workspace. Active calibration state drives the solver.

3. **IK solver** (`openarm_ik_solver/`, `server/ik_service.py`) — JAX-based nonlinear
   optimization. Warm-starts from previous solution (responsive) or home pose
   (deterministic). Solves bimanual IK with collision avoidance at 50 Hz.

4. **ROS2 publisher** (`server/ros2_publisher.py`) — publishes per-arm `JointState`
   and `PoseStamped` to standard topics. Thread-safe, runs in a daemon thread.

5. **WebSocket server** (`server/ws_handler.py`, `server/solver_loop.py`) — drives
   the 50 Hz solve loop and broadcasts snapshots to the browser monitoring UI.
   Serves the `webui-monitor/` static assets and provides `/ws` (snapshot stream)
   and `/ws/arm` (joint commands for motor control).

## Configuration

Edit `config/vr.yaml` to match your VR bridge topics and coordinate transform:

```yaml
ros2:
  topics:
    left_pose: "/vr/left_pose"
    right_pose: "/vr/right_pose"
    head_pose: "/vr/head_pose"
    left_buttons: "/vr/left_buttons"
    right_buttons: "/vr/right_buttons"

axis_mapping:
  native_to_intermediate:       # VR-native → robot base frame
    - [0.0, 0.0, -1.0]
    - [-1.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0]
  position_scale: 1.0
```

Edit `config/robot.yaml` to match your physical robot's home position and joint names.

Edit `config/solver_smooth.yaml` (default) to tune IK solver behavior.
Override with `--solver-config`:

## ROS2 Topic Conventions

This adaptor follows the [robot interface convention](../../README.md):

| Requirement | Status |
|-------------|--------|
| Message type: `JointState`, not `Float32MultiArray` | Followed |
| `header.stamp` set to solve timestamp | Followed |
| `header.frame_id` set to `base_link` | Followed |
| Joint `name[]` from URDF | Followed |
| Per-arm topics (`/arm/left/*`, `/arm/right/*`) | Followed |
| QoS: BEST_EFFORT, VOLATILE, KEEP_LAST | Followed |
| End-effector target pose published as `PoseStamped` | Followed |

## Session YAML (Data Collection Service)

```yaml
streams:
  - name: "left_joint_commands"
    source: robot
    topic: "/arm/left/joint_commands"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "position"
        type: sequence
        required: true

  - name: "right_joint_commands"
    source: robot
    topic: "/arm/right/joint_commands"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "position"
        type: sequence
        required: true

  - name: "left_target_pose"
    source: robot
    topic: "/arm/left/target_pose"
    message_type: "geometry_msgs/PoseStamped"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "pose.position.x"
        type: float64
        required: true
      - path: "pose.position.y"
        type: float64
        required: true
      - path: "pose.position.z"
        type: float64
        required: true
      - path: "pose.orientation.x"
        type: float64
        required: true
      - path: "pose.orientation.y"
        type: float64
        required: true
      - path: "pose.orientation.z"
        type: float64
        required: true
      - path: "pose.orientation.w"
        type: float64
        required: true
```

## Frontend (Self-Contained)

The `webui-monitor/` folder is a standalone static web app — copy it anywhere and serve
with any HTTP server. It connects to the IK solver via WebSocket at the configured URL.

```
webui-monitor/
├── config.js              ← WS_URL and URDF_PATH configuration
├── web_pages/             ← index.html, app.js, styles.css
├── 3d_assets/             ← scene_markers.js, URDF model, meshes
└── vendor/                ← Three.js, URDF loader
```

To deploy the webui-monitor separately:

```bash
cp -r webui-monitor/ /any/path/
cd /any/path/webui-monitor/web_pages
# Edit ../config.js to set WS_URL to your IK server
python3 -m http.server 8080
# Open http://localhost:8080
```

The webui-monitor auto-reconnects every 2 seconds if the IK server is not yet running.

## Deploying the Full Pipeline

This directory is a self-contained bundle. The target machine needs only Python 3.10
and x86_64 Linux — no network access, no `pip install`, no `git clone`.

```bash
# Create portable tarball
tar -chzf openarm-ik-adaptor.tar.gz \
    --exclude='__pycache__' --exclude='*.pyc' \
    openarm-ik-ros2-adaptor/

# On target machine
tar -xzf openarm-ik-adaptor.tar.gz
cd openarm-ik-ros2-adaptor
bash launch/start.sh --seed-caches
bash launch/start.sh
```

## Reference

- [Robot Interface Convention](../../README.md) — JointState + PoseStamped spec
- [VR Bridge Adaptor](../../../teleoperation_interface/vr-standard-ros2-bridge-adaptor/) — teleop data source
- [Data Collection Service](../../../data_collection_service/README.md) — session YAML format
- [Adapter Contract Guide](../../../../memodocs/adapter_contract_guide.md) — 3-layer contract rules

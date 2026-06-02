# OpenArm 3D Monitor

Standalone browser-based 3D monitoring UI. Subscribes to ROS2 topics
(arm joints, targets, VR poses) and streams snapshots to connected
browsers via WebSocket. Completely decoupled from the IK solver.

## Subscribed ROS2 Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/arm/left/joint_commands` | `JointState` | Left arm joint angles |
| `/arm/right/joint_commands` | `JointState` | Right arm joint angles |
| `/arm/left/target_pose` | `PoseStamped` | Left end-effector target |
| `/arm/right/target_pose` | `PoseStamped` | Right end-effector target |
| `/vr/head_pose` | `PoseStamped` | VR headset pose |
| `/vr/left_pose` | `PoseStamped` | Left controller pose |
| `/vr/right_pose` | `PoseStamped` | Right controller pose |

All topic names are configurable in `config/monitor.yaml`.

## Quick Start

```bash
cd openarm-monitor
source /opt/ros/humble/setup.bash
python3 monitor_server.py
```

Open `http://localhost:5201` in a browser.

## Configuration

Edit `config/monitor.yaml` to change topics, port, or broadcast rate:

```yaml
ros2_topics:
  left_joint_commands: "/arm/left/joint_commands"
  right_joint_commands: "/arm/right/joint_commands"
  # ... etc

server:
  port: 5201
  broadcast_hz: 20

python_env:
  type: system  # or conda / venv
```

## Files

```
openarm-monitor/
├── monitor_server.py        # ROS2 subscriber + HTTP/WS server
├── config/
│   └── monitor.yaml         # Topic names, port, broadcast rate, python env
├── launch/
│   └── start.sh             # Config-driven launcher
├── web/                     # 3D monitoring UI (THREE.js)
├── 3D_assets/               # URDF + mesh files
└── README.md
```

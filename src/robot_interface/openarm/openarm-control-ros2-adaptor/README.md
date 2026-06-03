# OpenArm Controller

Arm state publishing and motor control for the OpenArm bimanual robot.
Deploy independently of camera streams.

## Contents

| Script | Role | ROS2 required? |
|--------|------|---------------|
| `arm_state_publisher.py` | Reads joints via CAN bus → publishes `JointState` to ROS2 | Yes |
| `arm_controller.py` | Receives IK commands via WebSocket → drives motors | No |

## Published ROS2 Topics

| Topic | Type | Content |
|-------|------|---------|
| `/arm/left/joint_state` | `sensor_msgs/JointState` | Left arm: 7 joints + gripper (radians) |
| `/arm/right/joint_state` | `sensor_msgs/JointState` | Right arm: 7 joints + gripper (radians) |

Each `JointState` includes `header.stamp`, `header.frame_id=base_link`, and `name[]`
from `config/robot.yaml`.

## Quick Start

**Launch script** (starts controller + publisher together):

```bash
cd openarm-control-ros2-adaptor
bash launch/start.sh
```

Calibrates, homes, waits for ENTER, then streams motor commands and publishes
`JointState` to ROS2.

**Standalone — Arm state publisher:**

```bash
cd openarm-control-ros2-adaptor
source /opt/ros/humble/setup.bash
python3 arm_state_publisher.py
```

**Standalone — Arm controller** (requires IK adaptor running on port 5200):

```bash
cd openarm-control-ros2-adaptor
python3 arm_controller.py --ws-uri ws://localhost:5200/ws/arm
```

## Prerequisites

The lerobot SDK is vendored in `lerobot/` — no separate install needed.
`LEROBOT_SRC` auto-detects from the adaptor directory.

| Requirement | Check | Install |
|-----------|-------|---------|
| Python 3.10+ | `python3 --version` | Activate your env (conda/venv) before running |
| ROS2 Humble | `source /opt/ros/humble/setup.bash` | Only needed for `arm_state_publisher.py` |
| SocketCAN | `ip link show can0` | `sudo ip link set can0 type can bitrate 1000000; sudo ip link set up can0` |
| IK adaptor running | `curl -s http://localhost:5200/ws/arm` | Start `openarm-ik-ros2-adaptor` first |
| Python packages | `python3 -c "import numpy, yaml, websockets"` | `pip install numpy pyyaml websockets` |
| URDF meshes (optional) | `ls urdf/openarm_bimanual_pybullet.urdf` | For gravity compensation — copy from `openarm_description` repo |

## Python Environment

Activate your environment before running. The scripts use `python3` from PATH:

```bash
# conda
source ~/miniforge3/etc/profile.d/conda.sh && conda activate lerobot

# venv
source ~/venvs/myenv/bin/activate
```

## Configuration

Edit `config/robot.yaml`: CAN bus ports, joint names, home position, Kp/Kd gains,
publish rate, safety clamps, URDF path for gravity compensation, WebSocket URI.

## Session YAML

```yaml
streams:
  - name: "left_joint_state"
    source: robot
    topic: "/arm/left/joint_state"
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

  - name: "right_joint_state"
    source: robot
    topic: "/arm/right/joint_state"
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
```

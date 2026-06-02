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

## Python Environment

The launch script uses `python_env` in `config/robot.yaml` to find the right
Python interpreter. Three modes are supported:

**conda** (default):
```yaml
python_env:
  type: conda
  conda_home: "~/miniforge3"
  conda_env: "lerobot"
```

**venv:**
```yaml
python_env:
  type: venv
  venv_path: "~/venvs/myenv"
```

**system** (use whatever is on PATH):
```yaml
python_env:
  type: system
```

When running scripts directly (not via `launch/start.sh`), activate the
environment manually beforehand.

## Configuration

Robot parameters in `config/robot.yaml`: CAN bus, joint names, home position,
Kp/Kd gains, publish rate, safety clamps, URDF path for gravity compensation,
and `python_env` (see above).

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
      depth: 10
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
      depth: 10
    fields:
      - path: "position"
        type: sequence
        required: true
```

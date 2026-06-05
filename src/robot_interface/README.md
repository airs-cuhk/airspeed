# Robot Interface — Data Convention

This document defines the data conventions that **any robot arm or actuation system**
must follow for the data collection service to correctly record its output.

Write a ROS2 publisher that reads your robot's SDK and publishes messages matching these
conventions. The included adaptors serve as reference implementations. The data collection
service subscribes to whatever topics the session YAML declares — it has no baked-in
knowledge of specific robots.

## Launch

```bash
cd robot_interface
bash run_global_config.sh                    # starts the configured adaptor
```

`global_config.yaml` points to the active adaptor and optionally a Python interpreter.
`run_global_config.sh` reads it and starts all listed adaptors concurrently. Uses `python3` from PATH.

```yaml
# global_config.yaml
adaptor: "openarm/openarm-control-ros2-adaptor"
# Uses python3 from PATH — activate your env before running
```

To add a new robot:

1. Create `<your-adaptor>/` with a `launch/start.sh` script
2. Change `global_config.yaml` → `adaptor: "your-adaptor"`
3. Optionally set `python:` to your environment's interpreter
4. Run `bash run_global_config.sh`

## Two-Layer Configuration

The project separates what gets recorded from how hardware is configured. Two YAML files,
two responsibilities, no source code edits.

| Layer | Location | Owned By | What It Controls |
|-------|----------|----------|-----------------|
| Session YAML | `data_collection_service/config/` | Data collection service | Topics to subscribe to, message types, field contracts, recording control mode |
| Interface YAML | `<adaptor>/config/` (each adaptor owns its config) | Robot driver | Device connection, joint definitions, publish rate, calibration |

### What the Session YAML Controls (Data Pipeline)

Which ROS2 topics to record, what message types they carry, which fields to extract,
QoS settings, and how recording starts/stops. This file is hardware-agnostic — it
declares "subscribe to `/arm/joint_states` of type `JointState`" without knowing
what robot produces that topic.

### What the Interface YAML Controls (Hardware Hyperparameters)

Device-specific settings that change when you swap hardware but the data pipeline stays
the same:

```yaml
# <your-adaptor>/config/my_robot.yaml
robot:
  type: "my_robot"               # SDK driver to use
  ip: "192.168.1.100"            # connection address
  port: 5001
joints:
  names: ["joint_1", ..., "joint_6"]   # which joints exist
  direction: [1, 1, 1, -1, 1, 1]      # per-joint sign correction
  zero_offset: [0.0, 0.0, ...]         # home position offset
publish:
  topic: "/arm/joint_states"     # where to publish (must match session YAML)
  rate_hz: 50                    # how fast to publish
```

Edit this file to match your robot — no source code changes needed.

### Why Two Layers

The session YAML answers "what data do I want to record?" The interface YAML answers
"how do I talk to this specific robot?" When you swap one robot for another, only the
interface YAML changes. The session YAML and the data collection pipeline stay the same.

## What the Data Collection Service Expects

The collector records two kinds of data from a robot:

| Data Category | Purpose | Required Message Type |
|--------------|---------|----------------------|
| Joint state | Position, velocity, effort for each actuated degree of freedom | `sensor_msgs/JointState` |
| End-effector pose | Cartesian position and orientation of the tool center point | `geometry_msgs/PoseStamped` |

You can publish one or both. The session YAML declares which topics to subscribe to —
omit the ones your robot does not provide.

## JointState Convention

`sensor_msgs/JointState` is the standard ROS2 message for articulated state. It carries
everything the collector and downstream consumers need in one message.

```python
from sensor_msgs.msg import JointState

msg = JointState()
msg.header.stamp = acquisition_timestamp      # from robot SDK, NOT ros clock
msg.header.frame_id = "base_link"             # or your robot's root frame
msg.name = ["joint_1", "joint_2", ...]         # names identify WHICH joint
msg.position = [0.1, -0.5, ...]                # radians or meters (be consistent)
msg.velocity = [0.0, 0.01, ...]                # rad/s or m/s
msg.effort = [0.0, 1.5, ...]                   # N·m or N
```

### Why JointState and not Float32MultiArray

| JointState provides | Float32MultiArray lacks |
|---------------------|------------------------|
| `header.stamp` — acquisition timestamp | No header at all |
| `name[]` — which value is which joint | No identification |
| `velocity[]` — joint velocities | Single data array only |
| `effort[]` — joint torques/forces | Single data array only |
| `float64` precision | `float32` precision |

Standard ROS2 tools (rviz2, PlotJuggler, `joint_state_broadcaster`) consume JointState
natively. Float32MultiArray requires custom adapters everywhere downstream.

### Joint Count and Naming

There is no fixed joint count. Your robot may have 3, 6, 7, or 20 joints — the adapter
auto-detects the dimension from the first frame. Joint names are whatever your robot
reports. The convention is lowercase with underscores: `joint_1`, `shoulder_pan_joint`,
`linear_axis_z`.

### Units

Use SI units consistently across all messages from the same robot:

| Quantity | Unit |
|----------|------|
| Position (revolute) | radians |
| Position (prismatic) | meters |
| Velocity | rad/s or m/s |
| Effort | N·m (revolute) or N (prismatic) |

## PoseStamped Convention

`geometry_msgs/PoseStamped` is the standard ROS2 message for a 6-DOF pose with timestamp
and reference frame.

```python
from geometry_msgs.msg import PoseStamped

msg = PoseStamped()
msg.header.stamp = acquisition_timestamp
msg.header.frame_id = "base_link"             # reference frame for this pose
msg.pose.position.x = 0.45                    # meters
msg.pose.position.y = 0.12
msg.pose.position.z = 0.33
msg.pose.orientation.x = 0.0                  # quaternion (xyzw), normalized
msg.pose.orientation.y = 0.0
msg.pose.orientation.z = 0.707
msg.pose.orientation.w = 0.707
```

### Coordinate Frame

Use the ROS convention: X-forward, Y-left, Z-up. If your robot uses a different
convention, transform before publishing. The data collection service records poses
as-is — it does not apply coordinate transforms.

### Orientation

Must be a normalized quaternion in xyzw order. This is the ROS2 default. If your
robot reports Euler angles or rotation matrices, convert before publishing.
A quaternion with norm ≈ 0 will be rejected by the boundary validator.

## Header Timestamp Rule

`header.stamp` must be the **acquisition time** — when the robot measured the state.
Not when your code published it, not when the ROS2 executor dispatched the callback.

```python
# Correct — hardware timestamp
msg.header.stamp = robot.get_state_timestamp()

# Wrong — adds unknown latency between measurement and recording
msg.header.stamp = self.get_clock().now().to_msg()
```

If your robot SDK does not expose acquisition timestamps, set `time_domain: ros_receive`
in the session YAML for that stream. The collector will use the message arrival time as
a fallback. This is less accurate but workable.

## Topic Naming Convention

Use semantic namespaces. The collector has no fixed topic names — the session YAML
declares what to subscribe to.

```
/arm/joint_states          # single arm
/arm/left/joint_states     # dual arm, left side
/arm/right/cartesian_pose  # dual arm, right side
/torso/joint_states        # torso/waist joint
/head/joint_states         # pan-tilt head
```

Keep the pattern: `/<subsystem>/<data_type>` or `/<subsystem>/<instance>/<data_type>`.

## QoS Convention

| Data Type | Reliability | Durability | History | Depth |
|-----------|------------|------------|---------|-------|
| Joint state (high-freq) | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |
| End-effector pose (high-freq) | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |

High-frequency state data prefers BEST_EFFORT — losing an occasional message is better
than building a backlog of delayed ones.

## Session YAML Examples

### Revolute multi-axis arm

```yaml
streams:
  - name: "joint_states"
    source: robot
    topic: "/arm/joint_states"
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
      - path: "velocity"
        type: sequence
        required: false
      - path: "effort"
        type: sequence
        required: false

  - name: "end_effector_pose"
    source: robot
    topic: "/arm/cartesian_pose"
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

### Prismatic single-axis (linear rail, lead screw)

```yaml
streams:
  - name: "linear_axis"
    source: robot
    topic: "/axis/state"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "position"      # single value in meters
        type: sequence
        required: true
```

### Robot with only joint feedback (no cartesian pose available)

```yaml
# Just declare the joint states stream. Omit the pose stream entirely.
streams:
  - name: "joint_states"
    source: robot
    topic: "/robot/joint_states"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    ...
```

## Reference

- [ROS2 Data Stream Standards](../../memodocs/ros2-data-stream-standards.md) — IDL types, common_interfaces reference, ros2_control conventions
- [Adapter Contract Guide](../../memodocs/adapter_contract_guide.md) — 3-layer contract, closed type vocabulary, procedure for new modalities
- [Pipeline Mapping Reference](../../memodocs/pipeline_mapping_debug_reference.md) — 8-hop data flow through the collector
- [Data Collection Service](../data_collection_service/README.md) — core architecture, session YAML format

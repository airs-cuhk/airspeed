# Teleoperation Interface — Data Convention

This document defines the data conventions that **any operator input device** must
follow for the data collection service to correctly record its output.

Write a ROS2 publisher that reads your device SDK and publishes messages matching these
conventions. The included adaptors serve as reference implementations. The data collection
service subscribes to whatever topics the session YAML declares — it has no baked-in
knowledge of specific devices.

## Launch

```bash
cd teleoperation_interface
bash run_global_config.sh                    # starts the configured adaptor
```

`global_config.yaml` points to the active adaptor and optionally a Python interpreter.
`run_global_config.sh` reads it and starts all listed adaptors concurrently. Uses `python3` from PATH.

```yaml
# global_config.yaml
adaptor: "vr-standard-ros2-bridge-adaptor"
# Uses python3 from PATH — activate your env before running
```

To add a new device:

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
| Interface YAML | `<adaptor>/config/` (each adaptor owns its config) | Teleop driver | Device type, coordinate transform, button channel mapping, publish rate |

### What the Session YAML Controls (Data Pipeline)

Which ROS2 topics to record, what message types they carry, which fields to extract,
QoS settings, and whether recording is controlled via browser UI, CLI service, or
device binding (a button on the operator device). This file is hardware-agnostic — it
declares "subscribe to `/operator/right/pose` of type `PoseStamped`" without knowing
what device produces that topic.

### What the Interface YAML Controls (Hardware Hyperparameters)

Device-specific settings that change when you swap hardware but the data pipeline
stays the same:

```yaml
# <your-adaptor>/config/my_device.yaml
device:
  type: "vr_controller"          # SDK driver to use
  ip: "0.0.0.0"                  # listen address (for network devices)
  port: 8080
coordinate_transform:             # device-native → ROS standard
  matrix: [[0, 0, 1],
           [-1, 0, 0],
           [0, -1, 0]]
publish:
  pose_topic: "/operator/right/pose"
  buttons_topic: "/operator/right/buttons"
  rate_hz: 30
button_map:                       # which physical channel is which
  trigger: 0
  grip: 1
  primary: 2
  secondary: 3
  menu: 5
```

Edit this file to match your device — no source code changes needed.

### Why Two Layers

The session YAML answers "what data do I want to record?" The interface YAML answers
"how do I talk to this specific device?" When you swap a VR controller for a joystick
or foot pedal, only the interface YAML changes. The session YAML and the data collection
pipeline stay the same.

## What the Data Collection Service Expects

The collector records two kinds of data from an operator input device:

| Data Category | Purpose | Required Message Type |
|--------------|---------|----------------------|
| 6-DOF pose | Position and orientation of a tracked point (hand, stylus, head, etc.) | `geometry_msgs/PoseStamped` |
| Discrete inputs | Button presses, trigger values, switch states | `std_msgs/Float32MultiArray` |

Any device that can produce one or both of these is compatible. The session YAML
declares which topics to subscribe to — declare only the ones your device provides.

## PoseStamped Convention

`geometry_msgs/PoseStamped` carries a tracked point's position and orientation with a
timestamp and reference frame. This is the same message type used by robots and sensors
— there is no separate "teleop pose" type.

```python
from geometry_msgs.msg import PoseStamped

msg = PoseStamped()
msg.header.stamp = device_timestamp              # acquisition time from device
msg.header.frame_id = "world"                    # or your tracking origin frame
msg.pose.position.x = tx                         # meters
msg.pose.position.y = ty
msg.pose.position.z = tz
msg.pose.orientation.x = qx                      # quaternion (xyzw), normalized
msg.pose.orientation.y = qy
msg.pose.orientation.z = qz
msg.pose.orientation.w = qw
```

### Coordinate Frame

Your device likely uses a coordinate frame different from ROS standard (X-forward,
Y-left, Z-up). Transform before publishing. Common cases:

| Device Frame | Transform to ROS |
|-------------|-----------------|
| Y-up, right-handed (VR, game engines) | X_ros = Z_dev, Y_ros = -X_dev, Z_ros = -Y_dev |
| Z-up, right-handed (some CAD tools) | Identity, or X_ros = Y_dev, Y_ros = -X_dev |
| Screen coordinates (2D input) | Map X/Y to a fixed plane, set Z = 0, orientation = identity |

The data collection service records poses as-is — it does not transform coordinates.

### Partial Pose (devices with fewer than 6 DOF)

Not every device tracks full 6-DOF. A 3-DOF stylus, a 2D joystick, or a single-axis
slider all produce partial poses.

- **3-DOF (position only)**: Set orientation to identity quaternion `(0, 0, 0, 1)`
- **2-DOF (XY only)**: Set Z to a fixed value, orientation to identity
- **1-DOF (single axis)**: Set other two axes to zero, orientation to identity
- **Orientation only (IMU-based tracker)**: Set position to `(0, 0, 0)`

The session YAML's `fields` list declares which components are `required: true` —
declare only the axes your device provides.

## Float32MultiArray Convention (Discrete Inputs)

`std_msgs/Float32MultiArray` carries button states, trigger values, switch positions,
or any other discrete operator input as a flat float array.

```python
from std_msgs.msg import Float32MultiArray

msg = Float32MultiArray()
msg.data = [0.0, 0.0, 1.0, 0.5, 0.0, 0.0]  # 6 input channels
```

### Value Convention

| Input Type | Value Range | Meaning |
|-----------|------------|---------|
| Binary button | 0.0 or 1.0 | 0.0 = released, 1.0 = pressed |
| Analog trigger | 0.0 to 1.0 | Continuous range from rest to fully engaged |
| 2-stage trigger | 0.0, 0.5, 1.0 | Discrete stages (rest, half-press, full-press) |
| Thumbstick axis | -1.0 to 1.0 | Continuous range, center = 0.0 |
| Switch / toggle | 0.0 or 1.0 | Two positions |
| Rotary encoder | accumulated float | Cumulative rotation count |

The collector stores these values as-is. It is your driver's responsibility to normalize
device-specific ranges (e.g., 0–255 → 0.0–1.0) before publishing.

### No Header

`Float32MultiArray` has no `header` field. The collector cannot read an acquisition
timestamp from this message type. For button streams, set `time_domain: ros_receive`
in the session YAML — the collector uses the ROS2 message arrival time instead.

### Channel Order

Your device defines the channel order. Document your mapping. Example for a two-hand
controller:

```
Channel 0: left trigger    (analog, 0.0–1.0)
Channel 1: left grip       (binary, 0.0/1.0)
Channel 2: left primary    (binary, 0.0/1.0)
Channel 3: left secondary  (binary, 0.0/1.0)
Channel 4: right trigger   (analog, 0.0–1.0)
Channel 5: right grip      (binary, 0.0/1.0)
Channel 6: right primary   (binary, 0.0/1.0)
Channel 7: right secondary (binary, 0.0/1.0)
```

The order matters — the downstream consumer of the HDF5 file needs to know which
channel is which. Document your mapping in your driver's README.

## Header Timestamp Rule

**For PoseStamped**: `header.stamp` must be the acquisition time from the device SDK.

```python
# Correct — device timestamp
msg.header.stamp = device.get_timestamp()

# Wrong — adds variable latency
msg.header.stamp = self.get_clock().now().to_msg()
```

**For Float32MultiArray**: No header exists. The session YAML must use `time_domain: ros_receive`.
This is the nature of the message type, not a flaw in your driver.

## Device Binding for Recording Control

The data collection service supports `device_binding` recording control mode, where a
physical input on the operator device starts and stops recording. Configure in the
session YAML:

```yaml
recording_control:
  mode: device_binding
  bindings:
    start:
      stream_name: "operator_buttons"  # the Float32MultiArray stream name in this session
      button_index: 0                  # which channel triggers recording
      threshold: 0.5                   # value above threshold = pressed
    stop:
      stream_name: "operator_buttons"
      button_index: 1
      threshold: 0.5
```

When the operator activates the designated channel, the collector toggles between
IDLE and RECORDING. No browser UI or CLI command needed.

## Topic Naming Convention

The collector has no fixed topic names. Use semantic namespaces that describe the
operator's body part or the device role:

```
/operator/right/pose            # right hand / primary controller
/operator/left/pose             # left hand / secondary controller
/operator/head/pose             # head-mounted display or viewpoint
/operator/right/buttons         # right-hand discrete inputs
/operator/left/buttons          # left-hand discrete inputs
/operator/pedal/buttons         # foot pedal inputs
/operator/stylus/pose           # 3D stylus or probe
```

## QoS Convention

| Data Type | Reliability | Durability | History | Depth |
|-----------|------------|------------|---------|-------|
| Pose (continuous tracking) | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |
| Buttons (discrete events) | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |

Poses and buttons both use BEST_EFFORT — a dropped frame is better than a delayed one.
For recording control via device_binding, session YAML QoS should use `reliability: reliable`
to ensure start/stop events are not dropped.

## Session YAML Examples

### Two-hand 6-DOF device with buttons

```yaml
streams:
  - name: "right_pose"
    source: teleop
    topic: "/operator/right/pose"
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

  - name: "right_buttons"
    source: teleop
    topic: "/operator/right/buttons"
    message_type: "std_msgs/Float32MultiArray"
    time_domain: ros_receive         # no header in Float32MultiArray
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: sequence
        required: true
```

### 3-DOF stylus (position only, no orientation tracking)

```yaml
streams:
  - name: "stylus_pose"
    source: teleop
    topic: "/operator/stylus/pose"
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
      # orientation fields omitted — device does not track them
```

### Foot pedal (buttons only, no pose)

```yaml
streams:
  - name: "pedal_buttons"
    source: teleop
    topic: "/operator/pedal/buttons"
    message_type: "std_msgs/Float32MultiArray"
    time_domain: ros_receive
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: sequence
        required: true
```

## Device Type Reference

The data collection service has no knowledge of what device produced the messages.
Any device that publishes PoseStamped and/or Float32MultiArray is compatible.

| Device | Produces | Notes |
|--------|----------|-------|
| VR/AR controller | 6-DOF pose + buttons | May need coordinate transform from Y-up to Z-up |
| Haptic stylus | 3–6 DOF pose + pressure | Map pressure to a Float32MultiArray channel |
| 3D mouse / SpaceMouse | 6-DOF delta | Integrate to absolute pose, or publish as velocity stream |
| Joystick / gamepad | 2–4 axes + buttons | Map axes to a fixed-plane PoseStamped, buttons to Float32MultiArray |
| Keyboard | Key events | Map keys to Float32MultiArray channels |
| Foot pedal | 1–3 switches | Publish as Float32MultiArray |
| Mobile / tablet app | Pose + touch events | Bridge over network to ROS2 topics |
| Motion capture marker | 3-DOF position | Publish as PoseStamped with identity orientation |

## Reference

- [ROS2 Data Stream Standards](../../memodocs/ros2-data-stream-standards.md) — IDL types, common_interfaces reference
- [Adapter Contract Guide](../../memodocs/adapter_contract_guide.md) — 3-layer contract, device_binding mode, closed type vocabulary
- [Pipeline Mapping Reference](../../memodocs/pipeline_mapping_debug_reference.md) — 8-hop data flow through the collector
- [Data Collection Service](../data_collection_service/README.md) — core architecture, session YAML format

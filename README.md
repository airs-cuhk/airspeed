# AIRSPEED v1.0 — Uniform Robot Data Collection Platform

Hardware-agnostic embodied AI data production platform. Collects multi-stream sensor and
actuation data from ROS2 topics and writes AIRS-standard HDF5 episode files.

## Architecture

```
                     ROS2 Topic Bus (DDS)
                             |
    +------------------------+------------------------+
    |                        |                        |
    v                        v                        v
teleoperation_interface  robot_interface       sensor_interface
  (contract doc)          (contract doc)         (contract doc)
    |                        |                        |
    |  PoseStamped            |  JointState            |  Image
    |  Float32MultiArray      |  PoseStamped           |  CameraInfo
    +------------------------+------------------------+
                             |
                             v
                   data_collection_service
                          (core)
                   YAML-driven pipeline
                   1:1 topic-to-stream contract
                             |
                             v
                     AIRS .h5 episodes
```

Each interface defines a data convention with a README and ships reference adaptors
that publish standard ROS2 topics. Hardware drivers follow the same convention —
as long as your publishers emit the declared message types on the declared topics,
the data collection service records them. It has no baked-in knowledge of specific hardware.

## Two-Layer Configuration System

The project separates configuration into two layers with distinct responsibilities.
Neither layer requires editing source code.

### Layer 1: Session YAML — What to Record

Located at `data_collection_service/config/session/`. This is the data collection
service's config. It declares **which ROS2 topics to subscribe to**, what message type
each topic carries, which fields to extract, QoS settings, and how recording is controlled.

```yaml
# config/session/my_session.yaml — declares topics and recording control
schema_version: "1.0"

session:
  name: "my_recording_session"
  task_id: "my_task"
  operator_id: "example_operator"
  recording_control:
    mode: manual_ui       # how to start/stop: service | manual_ui | device_binding
  devices:
    my_robot:
      device_id: robot-01
      role: robot

storage:
  root: data/episodes
  format: hdf5

streams:
  - name: "joint_states"
    source: robot
    topic: "/arm/joint_states"          # ← which ROS2 topic to subscribe to
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 10
    fields:                              # ← which fields to extract and validate
      - path: "position"
        type: sequence
        required: true
```

**Responsibility**: the data pipeline — topics, message types, field extraction, recording
state machine. This file is the sole source of truth for what gets recorded.

### Layer 2: Interface YAML — How to Talk to Hardware

One per adaptor (each adaptor owns its `config/` directory). These configure
**device-specific hyperparameters** that vary between hardware instances — things like
IP addresses, serial ports, calibration values, joint names, camera resolution, button
mappings, and coordinate transforms.

```yaml
# <your-adaptor>/config/my_robot.yaml — device hyperparameters
robot:
  ip: "192.168.1.100"
  port: 5001
joints:
  names: ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
  direction: [1, 1, 1, -1, 1, 1]        # per-joint sign flip
publish:
  topic: "/arm/joint_states"
  rate_hz: 50
```

```yaml
# <your-adaptor>/config/my_camera.yaml — device hyperparameters
camera:
  serial_number: "241322301478"
streams:
  color:
    width: 640
    height: 480
    fps: 30
    encoding: "jpeg"
```

**Responsibility**: the hardware layer — connection details, device-specific settings,
publish rates. These values change when you swap hardware but the data pipeline stays
the same.

### Why Two Layers

The session YAML answers "what data do I want to record?" The interface YAML answers
"how do I talk to this specific hardware?" Swapping hardware means editing one YAML file,
not changing source code.

### Launching

Each interface has `global_config.yaml` pointing to the active adaptor, and
`run_global_config.sh` to start it. The data collection service is started separately:

```bash
cd src
bash teleoperation_interface/run_global_config.sh   # VR bridge
bash robot_interface/run_global_config.sh           # IK + arm control
bash sensor_interface/run_global_config.sh          # Cameras
```

## Cross-Machine Data Pipelines

ROS2 (DDS) is the local data bus on each machine. For cross-machine transport across
firewalls or separate subnets, use a relay bridge rather than configuring DDS routing.

### ROS2 on Each Machine, Relay Between Them

```
Machine A (hardware)                        Machine B (collector)
┌──────────────────────┐                  ┌──────────────────────────┐
│  drivers             │                  │  data_collection_service │
│     │                │                  │       ▲                  │
│     ▼                │                  │       │                  │
│  ROS2 topics (DDS)   │    WS / SSH      │  ROS2 topics (DDS)       │
│  shared memory       │                  │  shared memory           │
│     │                │                  │       ▲                  │
│     ▼                │                  │       │                  │
│  relay node ─────────┼── TCP tunnel ────┼──→ relay node            │
│  (ROS2→JSON→WS)      │                  │  (WS→JSON→ROS2)          │
└──────────────────────┘                  └──────────────────────────┘
```

The relay subscribes to local ROS2 topics on machine A, serializes to JSON, and sends
over a WebSocket connection (typically tunneled through SSH). A matching relay on machine B
deserializes and republishes to local ROS2 topics. The collector on machine B sees standard
ROS2 topics — it has no knowledge the data originated on another machine.

### Why Not Pure WebSocket End-to-End

| Concern | ROS2 (per machine) + WS relay (between machines) | Pure WebSocket end-to-end |
|---------|--------------------------------------------------|--------------------------|
| Local transport speed | Shared memory (10 MB at 30 Hz, zero degradation) | TCP loopback (kernel stack on every frame) |
| Message types | `JointState`, `PoseStamped`, `Image` — standard, generated | Must define JSON schemas for every type |
| QoS per stream | One YAML line | Must implement reliability, buffering, backpressure by hand |
| Tooling | rviz2, PlotJuggler, ros2bag work natively | Nothing in the robotics ecosystem speaks WS |
| Cross-machine | WS tunnel through SSH (firewall-friendly) | WS works, but you sacrificed all of the above to get here |

### Timestamp Fidelity

The relay must preserve the original `header.stamp` from hardware messages. The collector
uses `time_domain: ros_header` and reads the hardware acquisition time — not the relay
arrival time. As long as the relay copies headers verbatim, cross-machine latency does
not affect recorded timestamps.

### When ROS2 Works Directly

On a LAN where machines share a subnet and multicast is allowed, ROS2 discovery works
transparently — the collector can subscribe to topics on machine A directly with no relay.
The relay is only needed when firewalls, NAT, or separate subnets prevent direct DDS
communication.

## ROS2 Topic Publishing Contract

Your publishers MUST follow these rules for the collector to correctly record data.

### 1. Use Standard Message Types

| Data | Message Type | Example Topic |
|------|-------------|---------------|
| Pose (position + orientation) | `geometry_msgs/PoseStamped` | `/vr/head_pose` |
| Joint states (position/velocity/effort) | `sensor_msgs/JointState` | `/arm/joint_states` |
| Numeric arrays (buttons, tactile) | `std_msgs/Float32MultiArray` | `/vr/buttons` |
| Camera image | `sensor_msgs/Image` | `/camera/color` |
| Point cloud | `sensor_msgs/PointCloud2` | `/camera/points` |
| IMU | `sensor_msgs/Imu` | `/imu` |

### 2. Every Message MUST Have a Header Timestamp

```python
msg.header.stamp = acquisition_timestamp   # hardware timestamp, NOT ros clock
msg.header.frame_id = "base_link"
```

Messages without headers (e.g., `Float32MultiArray`) must use `time_domain: ros_receive`
in the session YAML — the collector uses arrival time as a fallback.

### 3. One Logical Signal Per Topic

```
Correct:  /arm/joint_states (JointState), /arm/pose (PoseStamped)   — separate
Wrong:    /arm/state (custom message with joints + pose)             — multiplexed
```

### 4. Use Semantic Namespaces

```
/arm/left/joint_states     — namespace by hardware, semantic name
/camera/color/image_raw    — standard REP-2001 convention
```

### 5. QoS Declaration

| Data Type | Reliability | Durability | History | Depth |
|-----------|------------|------------|---------|-------|
| High-freq state | BEST_EFFORT | VOLATILE | KEEP_LAST | 1-10 |
| Camera images | BEST_EFFORT | VOLATILE | KEEP_LAST | 1-5 |
| Button events | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |

### 6. Prefer JointState Over Float32MultiArray

`sensor_msgs/JointState` provides `header.stamp`, `name[]` (joint identification),
`velocity[]`, `effort[]`, and `float64` precision. `Float32MultiArray` has none of these.

## Quick Start

```bash
cd airspeed-main-v1.0/src

# 1. Start hardware publishers (one terminal each):
bash teleoperation_interface/run_global_config.sh      # VR bridge
bash robot_interface/run_global_config.sh              # IK + arm control
bash sensor_interface/run_global_config.sh             # Cameras

# 2. Start the data collector:
cd data_collection_service
source /opt/ros/humble/setup.bash
PYTHONPATH="core:tools:$PYTHONPATH" /usr/bin/python3.10 \
  -m core.runtime.ros2_collection_node \
  --session-config config/session/session_vr_ik_robot_button_control.yaml \
  --output-dir data/episodes \
  --operator-ui-port 8765
```

Open `http://localhost:8765` for the recording dashboard.

Run tests:

```bash
cd data_collection_service && PYTHONPATH="core:tests:tools" python3 -m pytest tests/ -q
# 55 tests, ~1.4s, no ROS2 needed for 54 of 55
```

## Project Structure

```
airspeed-main-v1.0/
├── README.md                           # This file
├── memodocs/
│   ├── ros2-data-stream-standards.md   # ROS2 message types, conventions, compliance audit
│   └── adapter_contract_guide.md       # Rules for YAML contracts, adapters, new modalities
└── src/
    ├── data_collection_service/        # CORE — YAML-driven multi-stream recorder
    │   ├── config/session/             #   Session YAML profiles
    │   ├── launch/                     #   ROS2 launch file
    │   ├── tools/                      #   Mock publishers, dataset validator
    │   ├── tests/                      #   55 tests (unit + integration)
    │   └── core/                       #   Python package (adapters, runtime, storage, ...)
    ├── robot_interface/                # CONTRACT + config-driven launcher
    │   ├── global_config.yaml                 #   points to active robot adaptor
    │   ├── run_global_config.sh               #   reads config → launches adaptor
    │   ├── README.md
    │   └── openarm/                    #   OpenArm adaptors
    ├── sensor_interface/               # CONTRACT + config-driven launcher
    │   ├── global_config.yaml                 #   points to active sensor adaptor
    │   ├── run_global_config.sh               #   reads config → launches adaptor
    │   ├── README.md
    │   └── camera-stream-adaptor/      #   RealSense camera publisher
    └── teleoperation_interface/        # CONTRACT + config-driven launcher
        ├── global_config.yaml                 #   points to active teleop adaptor
        ├── run_global_config.sh               #   reads config → launches adaptor
        ├── README.md
        └── vr-standard-ros2-bridge-adaptor/  # VR bridge server
```

## Using Custom Hardware

1. Read the interface README for your hardware domain — it documents the ROS2 topic contract
2. Write a ROS2 publisher node that reads your hardware and publishes matching messages
3. Create a session YAML declaring your topics (or reuse an existing profile)
4. Start your publishers, then start the collector

The collector subscribes to whatever topics the YAML declares. There is no baked-in
knowledge of specific hardware, topic names, or message types.

## Getting Started After Clone

This repo contains only source code and configuration (~5 MB). Large dependencies
must be built or installed on the target machine.

### What Works Out of the Box

| Component | Prerequisites |
|-----------|--------------|
| `data_collection_service/` — full core pipeline | Python 3.10, numpy, h5py, PyYAML (`pip install numpy h5py pyyaml`) |
| `teleoperation_interface/` — VR bridge server | Python 3.10, aiohttp (`pip install aiohttp`) |
| `sensor_interface/` — camera stream publisher | Python 3.10, OpenCV (`pip install opencv-python`) |
| All launch scripts (`run_global_config.sh`) | Python 3.10 with PyYAML |

### What You Must Build

| Component | Size | How |
|-----------|------|-----|
| IK solver Python deps (`.pydeps/`) | ~700 MB | `pip install --target .pydeps jax[cpu] jaxlie pyroki yourdfpy aiohttp ...` |
| IK solver caches (`.cache/`) | ~260 MB | `cd openarm-ik-ros2-adaptor && bash launch/start.sh --seed-caches` |
| 3D meshes (`3D_assets/urdf/meshes/`) | ~190 MB | Clone `github.com/enactic/openarm_description`, copy `meshes/` into `3D_assets/urdf/` |

These are excluded from git via `.gitignore` — they are too large (1.1 GB total)
and contain platform-specific binaries.

### What You Must Configure

| File | What to change |
|------|---------------|
| `robot_interface/openarm/openarm-control-ros2-adaptor/config/robot.yaml` | CAN bus ports, home position, Kp/Kd gains, URDF path |
| `robot_interface/openarm/openarm-ik-ros2-adaptor/config/vr.yaml` | Axis mapping matrix (VR coordinate transform) |
| `robot_interface/global_config.yaml` | `python:` — set to conda/venv path or leave empty for auto-detect |
| `teleoperation_interface/vr-standard-ros2-bridge-adaptor/config/config.json` | Port, IP address |
| `data_collection_service/config/session/` | Session YAML — declare your ROS2 topics |

### Environment Variables

| Variable | Used By | Default |
|----------|---------|---------|
| `LEROBOT_SRC` | arm_controller, arm_state_publisher, camera_publisher | (empty — set to lerobot source path if not on PYTHONPATH) |
| `PYTHON_BIN` | all launch scripts | auto-detected (`/usr/bin/python3.10`) |
| `DATA_COLLECTION_SERVICE_ROOT` | data collection service | auto-detected |

### System Requirements

- **OS**: Ubuntu 22.04+ (x86_64)
- **ROS2**: Humble (`/opt/ros/humble/setup.bash`)
- **Python**: 3.10 (ROS2 Humble requires this exact version)
- **CAN**: SocketCAN interfaces (`can0`, `can1`) for arm control
- **USB 3.2**: Required for multi-stream RealSense cameras

## Reference Documents

- [ROS2 Data Stream Standards](memodocs/ros2-data-stream-standards.md) — message types, conventions, compliance audit for `vr-ik-robot-data-collection`
- [Adapter Contract Guide](memodocs/adapter_contract_guide.md) — 3-layer contract (transport/payload/storage), closed type vocabulary, step-by-step procedure for adding new modalities
- [Robot Interface Contract](src/robot_interface/README.md) — JointState + PoseStamped topic spec
- [Sensor Interface Contract](src/sensor_interface/README.md) — Image + CameraInfo topic spec
- [Teleoperation Interface Contract](src/teleoperation_interface/README.md) — PoseStamped + Float32MultiArray topic spec

# AIRSPEED: An Open Source Data Production Platform for Embodied Artificial Intelligence

[![DOI](https://img.shields.io/badge/DOI-10.1145%2F3806827-blue)](https://doi.org/10.1145/3806827)

Welcome to the AIRSPEED GitHub repository!

---

## Introduction

Embodied Artificial Intelligence (EAI) data acquisition is widely recognized as
one of the key focuses in the development of embodied intelligence today. A
critical reason is that Scaling Laws are still considered effective in the field
of embodied intelligence — the better the performance of the model, the higher
the demand for training data. However, data acquisition encounters difficulties
in practice:

* The high cost of collecting a large amount of high-quality human demonstration
  and robot perception data is difficult to bear.
* It is difficult to collect data under a rich variety of training scenarios,
  tasks, and robot model categories.
* In the process of data collection, there are no corresponding standards or
  theories to guide whether the collected data has improved the quality of the
  dataset, whether it has increased the richness of the dataset, and by how much.

To address the above issues, we propose the open-source embodied intelligence
data production platform AIRSPEED. AIRSPEED has the following features:

* **Hardware-software decoupling** — Reduces software costs through an
  open-source platform. Any device that publishes standard ROS2 messages is
  compatible; the data pipeline is configured entirely through YAML files
  without source code edits.
* **Multiple devices supporting** — Supports a variety of data acquisition
  technologies (VR controllers, cameras, robotic arms) through a uniform
  ROS2 topic contract, ensuring a rich variety of scenarios, tasks, and models.
* **Dataset quality assurance** — Boundary validation rejects malformed data
  (NaN/Inf) at ingest time. Post-collection format conversion (Parquet, Zarr,
  LeRobot v3) enables seamless integration with ML training pipelines.
* **Extensible adaptor architecture** — Each interface defines a topic
  convention. The bundled adaptors are reference implementations; new devices
  are added by creating a new adaptor folder following the same contract.

---

## Architecture

AIRSPEED v2.0 consists of three interfaces and one core service:

* The **Teleoperation Interface** receives data from any operator input device
  (VR controller, joystick, foot pedal, etc.) and publishes standardized
  `PoseStamped` and `Float32MultiArray` messages to ROS2 topics.
* The **Robot Interface** converts teleoperation commands into control
  parameters for any robot arm and receives joint state feedback. Includes
  a JAX-based inverse kinematics solver and CAN bus motor control.
* The **Sensor Interface** receives data from any camera or environmental
  sensor and publishes `Image`, `CameraInfo`, and `Imu` messages.
* The **Data Collection Service** subscribes to declared ROS2 topics,
  validates every message against per-stream YAML contracts, and writes
  AIRS-standard HDF5 episode files. A post-storage conversion pipeline
  outputs Parquet, Zarr, or LeRobot v3 for ML training.

> **Note**: The current release (v2.0) includes Teleoperation Interface,
> Robot Interface, Sensor Interface, and Data Collection Service. Data
> generation from simulation environments and automated dataset construction
> are planned for future releases.

<!-- Banner placeholder: system concept + pipeline overview diagram -->
<p align="center">
  <img src="image/airspeed-banner.png" width="80%" alt="AIRSPEED System Overview">
</p>

---

## Quick Start

```bash
git clone https://github.com/StarChen-Cycler/airspeed-data-collection-zyc.git
cd airspeed-main-v1.0
```

### Environment Setup

Each sub-project has a `pyproject.toml` with its dependencies. Install with your
preferred tool:

```bash
# conda
conda create -n airspeed python=3.10
conda activate airspeed
pip install -e src/teleoperation_interface/vr-standard-ros2-bridge-adaptor/
pip install -e src/sensor_interface/camera-stream-adaptor/
pip install -e src/data_collection_service/

# uv
uv pip install -e src/teleoperation_interface/vr-standard-ros2-bridge-adaptor/
uv pip install -e src/sensor_interface/camera-stream-adaptor/
uv pip install -e src/data_collection_service/
```

The IK adaptor requires a JAX bundle (~700 MB). Install separately:

```bash
cd src/robot_interface/openarm/openarm-ik-ros2-adaptor
pip install --target .pydeps jax[cpu] jaxlie pyroki yourdfpy aiohttp
bash launch/start.sh --seed-caches
```

### Launch

Activate your environment before running — all scripts use `python3` from PATH.

### Teleoperation Interface
```bash
cd src/teleoperation_interface
bash run_global_config.sh
```

### Robot Interface
```bash
cd src/robot_interface
bash run_global_config.sh
```

### Sensor Interface
```bash
cd src/sensor_interface
bash run_global_config.sh
```

### Data Collection Service
```bash
cd src/data_collection_service
source /opt/ros/humble/setup.bash
bash run_global_config.sh
```

Open `http://localhost:8765` for the recording dashboard.
The IK adaptor monitoring UI is at `http://localhost:5200`.

---

## Project Structure

```
airspeed-main-v1.0/
├── README.md
└── src/
    ├── teleoperation_interface/             # CONTRACT: PoseStamped + Float32MultiArray
    │   ├── global_config.yaml               #   points to active adaptor
    │   ├── run_global_config.sh             #   reads config → launches adaptor
    │   └── vr-standard-ros2-bridge-adaptor/ #   HTTPS server → /vr/* ROS2 topics
    ├── robot_interface/                     # CONTRACT: JointState + PoseStamped
    │   ├── global_config.yaml
    │   ├── run_global_config.sh
    │   └── openarm/
    │       ├── robot_shared.yaml            #   Shared home positions (IK + control)
    │       ├── openarm-ik-ros2-adaptor/     #   VR→normalize→JAX IK→JointState (50 Hz)
    │       └── openarm-control-ros2-adaptor/#   CAN bus → joint state + motor control
    ├── sensor_interface/                    # CONTRACT: Image + CameraInfo
    │   ├── global_config.yaml
    │   ├── run_global_config.sh
    │   └── camera-stream-adaptor/           #   RealSense → JPEG → ROS2 Image
    └── data_collection_service/             # CORE: YAML-driven multi-stream recorder
        ├── global_config.yaml               #   session YAML, output dir, UI port
        ├── run_global_config.sh
        ├── config/                          #   Session YAML profiles
        ├── core/                            #   Python package
        │   ├── adapters/                    #     Generic message extraction
        │   ├── config/                      #     SessionConfig loader
        │   ├── contracts/                   #     WriterSample, enums
        │   ├── runtime/                     #     ROS2 node, state machine, UI
        │   ├── schema/                      #     Payload profiles + validator
        │   ├── storage/                     #     Chunked AIRS HDF5 writer
        │   └── validation/                  #     Post-collection HDF5 validator
        └── tools/                           #   Mock publishers, converters, validators
```

---

## Two-Layer Configuration

No source code edits to change hardware or recording behavior. Two YAML layers:

### Layer 1: Session YAML — What to Record

Located at `data_collection_service/config/`. Declares which ROS2 topics to
subscribe to, message types, field extraction rules, QoS, and recording control.

```yaml
schema_version: "1.0"
session:
  name: "my_session"
  recording_control:
    mode: manual_ui       # service | manual_ui | device_binding
storage:
  root: data/episodes
  format: hdf5
streams:
  - name: "joint_states"
    source: robot
    topic: "/arm/joint_states"
    message_type: "sensor_msgs/JointState"
    time_domain: ros_header
    qos: { reliability: best_effort, durability: volatile, history: keep_last, depth: 1 }
    fields:
      - { path: "position", type: sequence, required: true }
```

### Layer 2: Interface YAML — How to Talk to Hardware

Each adaptor owns its `config/` directory. Device-specific settings — IP, serial
ports, calibration, joint names, camera resolution, coordinate transforms — live
here. Swap hardware by editing one YAML file.

---

## ROS2 Topic Contract

Your publishers MUST follow these rules:

| Data | Message Type | Example Topic |
|------|-------------|---------------|
| Pose (position + orientation) | `geometry_msgs/PoseStamped` | `/vr/head_pose` |
| Joint states | `sensor_msgs/JointState` | `/arm/joint_states` |
| Numeric arrays (buttons) | `std_msgs/Float32MultiArray` | `/vr/buttons` |
| Camera image | `sensor_msgs/Image` | `/camera/color` |
| Point cloud | `sensor_msgs/PointCloud2` | `/camera/points` |
| IMU | `sensor_msgs/Imu` | `/imu` |

**Key rules**:
1. Every message with a header must carry hardware acquisition timestamps
2. One logical signal per topic — don't multiplex
3. Use semantic namespaces (`/arm/left/joint_states`)
4. Prefer `JointState` over `Float32MultiArray` for joint data

Full conventions are documented in each interface's README.

---

## Using Custom Hardware

1. Read the interface README for your domain — it documents the topic contract
2. Write a ROS2 publisher that reads your hardware and publishes matching messages
3. Create a session YAML declaring your topics (or reuse an existing profile)
4. Start your publishers, then start the collector

The collector has no baked-in knowledge of specific hardware, topic names, or
message types. Everything is declared in YAML.

---

## Distributed Data Collection Deployment

ROS2 (DDS) is the local data bus. For cross-machine transport across firewalls
or subnets, use a relay bridge (ROS2→JSON→WebSocket→JSON→ROS2) tunneled through
SSH, rather than configuring DDS routing directly. The relay preserves original
`header.stamp` timestamps so cross-machine latency does not affect recorded data.

---

## Post-Storage Conversion

AIRS HDF5 episodes can be converted to standard formats for ML training:

| Target | Tool | Best For |
|--------|------|----------|
| Parquet | `convert_h5_to_parquet.py` | Analytics, Pandas/DuckDB |
| Zarr | `convert_h5_to_zarr.py` | Cloud, multi-GPU |
| LeRobot v3 | `convert_h5_to_lerobot.py` | PyTorch, HF Hub |
| JSON Lines | `convert_h5_to_jsonl.py` | Debugging, inspection |

All converters are in `src/data_collection_service/tools/`. Each runs inline
validation against the source HDF5.

---

## System Requirements

| Requirement | Notes |
|-------------|-------|
| **OS** | Ubuntu 22.04+ (x86_64) |
| **ROS2** | Humble (`/opt/ros/humble/setup.bash`) |
| **Python** | 3.10 (activate your env before running; `python3` on PATH) |
| **CAN** | SocketCAN (`can0`, `can1`) for arm control |
| **USB 3.2** | Required for multi-stream RealSense cameras |
| **Dependencies** | Per sub-project `pyproject.toml` files; IK adaptor needs `.pydeps/` (~700 MB JAX bundle) |

---

## Reference Documents

- [Data Collection Service](src/data_collection_service/README.md) — core architecture, session YAML format, tools
- [Robot Interface](src/robot_interface/README.md) — JointState + PoseStamped topic spec
- [Sensor Interface](src/sensor_interface/README.md) — Image + CameraInfo topic spec
- [Teleoperation Interface](src/teleoperation_interface/README.md) — PoseStamped + Float32MultiArray topic spec
- [IK Adaptor Pipeline Mapping](src/robot_interface/openarm/openarm-ik-ros2-adaptor/PIPELINE_MAPPING.md) — 8-hop VR→joints trace with code snippets

---

## FAQ

**Q: Launch scripts won't run?**
Ensure `python3` is on PATH with `pyyaml` installed. Activate your conda/venv first.
Each script reports which dependency is missing.

**Q: No topic data or robot not responding?**
Check with `ros2 topic list` and `ros2 topic echo`. Verify the VR bridge is running
before the IK adaptor, and the IK adaptor before the arm controller.

**Q: Can I use a different robot or add new sensors?**
Yes. Write a ROS2 publisher matching the interface contract, declare your topics
in a session YAML, and start the collector. No changes to the core pipeline needed.

**Q: How do I share datasets?**
Convert HDF5 to LeRobot v3 format and push to HuggingFace Hub, or use Parquet/Zarr
for direct consumption in training pipelines.

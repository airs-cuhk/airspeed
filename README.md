# AIRSPEED v1.0 — Uniform Robot Data Collection Platform

<!-- Banner placeholder: system concept + pipeline overview diagram -->
<p align="center">
  <img src="image/airspeed-banner.png" width="80%" alt="AIRSPEED System Overview">
</p>

Hardware-agnostic embodied AI data production platform. Collects multi-stream sensor
and actuation data from ROS2 topics and writes AIRS-standard HDF5 episode files.

---

## Introduction

Embodied AI data acquisition faces three challenges: the high cost of collecting
large-scale human demonstrations, the difficulty of covering diverse scenarios and
robot models, and the lack of standards for measuring dataset quality.

AIRSPEED addresses these with:

- **Hardware-software decoupling** — open-source platform lowers software costs;
  any device that publishes standard ROS2 messages is compatible
- **Multi-device support** — VR controllers, cameras, robotic arms all connect
  through a uniform topic contract
- **YAML-driven pipeline** — no source code edits to change hardware, topics,
  or recording behavior; everything is declared in config files
- **Dataset quality tooling** — boundary validation (NaN/Inf rejection), JPEG
  integrity checks, post-collection format conversion (Parquet, Zarr, LeRobot)

The current release includes three interfaces and the data collection service.
Data generation (simulation) and automated dataset construction are planned
for future releases.

---

## Architecture

```
                      ROS2 Topic Bus (DDS)
                              |
     +------------------------+------------------------+
     |                        |                        |
     v                        v                        v
teleoperation_interface   robot_interface        sensor_interface
  PoseStamped              JointState              Image
  Float32MultiArray        PoseStamped             CameraInfo
     |                        |                        |
     +------------------------+------------------------+
                              |
                              v
                    data_collection_service
                     YAML-driven pipeline
                  adapter → validate → gate → buffer → HDF5
                              |
                              v
                      AIRS .h5 episodes
                              |
              +---------------+---------------+
              v               v               v
          Parquet           Zarr          LeRobot v3
       (post-storage conversion tools)
```

**Three interfaces** define ROS2 topic contracts. Each ships a reference adaptor
that publishes standard message types. As long as your publishers emit the declared
types on the declared topics, the data collection service records them — it has
no baked-in knowledge of specific hardware.

**One data collection service** subscribes to whatever topics the session YAML
declares, validates every message against per-stream contracts, and writes
AIRS-standard HDF5 episode files. Post-storage tools convert HDF5 to Parquet,
Zarr, or LeRobot v3 for ML training pipelines.

Detailed pipeline traces with code snippets and line numbers are in the
[pipeline mapping documents](#reference-documents).

---

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
bash run_global_config.sh
```

Open `http://localhost:8765` for the recording dashboard.
The IK adaptor monitoring UI is at `http://localhost:5200`.

Run tests:

```bash
cd data_collection_service && PYTHONPATH="core:tests:tools" python3 -m pytest tests/ -q
# 55 tests, ~1.4s, no ROS2 needed for 54 of 55
```

All launch scripts use `python3` from PATH — activate your environment
(conda/venv) before running. Each interface's `global_config.yaml` points
to the active adaptor; edit it to switch hardware.

---

## Project Structure

```
airspeed-main-v1.0/
├── README.md
├── memodocs/                                # Design & reference docs
│   ├── ros2-data-stream-standards.md        #   ROS2 message conventions, compliance
│   ├── adapter_contract_guide.md            #   YAML contract rules, new modalities
│   ├── pipeline_mapping_debug_reference.md  #   Full 8-hop HDF5 write path trace
│   └── pipeline_mapping_coverage_gaps.md    #   What the HOP docs don't cover
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
        ├── tests/                           #   55 tests (unit + integration)
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

Full conventions: [ROS2 Data Stream Standards](memodocs/ros2-data-stream-standards.md).

---

## Using Custom Hardware

1. Read the interface README for your domain — it documents the topic contract
2. Write a ROS2 publisher that reads your hardware and publishes matching messages
3. Create a session YAML declaring your topics (or reuse an existing profile)
4. Start your publishers, then start the collector

The collector has no baked-in knowledge of specific hardware, topic names, or
message types. Everything is declared in YAML.

---

## Cross-Machine Deployments

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

- [ROS2 Data Stream Standards](memodocs/ros2-data-stream-standards.md) — message types, conventions
- [Adapter Contract Guide](memodocs/adapter_contract_guide.md) — 3-layer contract, new modality procedure
- [Pipeline Mapping — Data Collection](memodocs/pipeline_mapping_debug_reference.md) — 8-hop HDF5 write path
- [Pipeline Mapping — IK Adaptor](src/robot_interface/openarm/openarm-ik-ros2-adaptor/PIPELINE_MAPPING.md) — 8-hop VR→joints trace
- [Robot Interface](src/robot_interface/README.md) — JointState + PoseStamped spec
- [Sensor Interface](src/sensor_interface/README.md) — Image + CameraInfo spec
- [Teleoperation Interface](src/teleoperation_interface/README.md) — PoseStamped + Float32MultiArray spec
- [Data Collection Service](src/data_collection_service/README.md) — core architecture, session YAML

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

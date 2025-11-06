# AIRSPEED: An Open-source Universal Data Production Platform for Embodied Artificial Intelligence
The paper will coming soon.

Welcome to AIRSPEED GitHub page!
   
## 🌏 Introduction
Embodied Artificial Intelligence (EAI) Data acquisition is widely recognized as one of the key focuses in the development of embodied intelligence today. A critical reason is that Scaling Laws are still considered effective in the field of embodied intelligence, which is reflected in data as the better the performance of the model, the higher the demand for training data. However, data acquisition encounters difficulties in practice, including

* The high cost of collecting a large amount of high-quality human demonstration and robot perception data is difficult to bear. 
* It is difficult to collect data under a rich variety of training scenarios, tasks, and robot model categories. 
* In the process of data collection, there are no corresponding standards or theories to guide whether the collected data has improved the quality of the dataset, whether it has increased the richness of the dataset, and by how much.

To address the above issues, we propose the open-source embodied intelligence data production platform AIRSPEED. AIRSPEED has the following features:

* Hardware-software decoupling - Reduces software costs through an open-source platform, helping to collect high-quality data at a low cost
* Multiple devices supporting - Supports a variety of data acquisition technologies to ensure a rich variety of scenarios/tasks/models, helping to comprehensively obtain highly generalized data
* Multiple simulation platform docking - Assists in quickly producing a large amount of data with synthetic samples
* Dataset automatic construction - Provides a method for constructing embodied intelligence datasets and offers a qualitative assessment method for the performance potential of the dataset

## 🏗️ Architecture
AIRSPEED consists of three interfaces and three services: Teleoperation Interface, Robot Interface, Sensor Interface, Data Collection Service, Data Generation Service, and Dataset Construction Service.

* The Teleoperation Interface is used to receive data from any teleoperation device and convert valid attitude control data. 
* The Robot Interface is used to convert attitude control data into control parameters for any robot and receive joint data from the robot. 
* The Sensor Interface is used to receive sensor data in any form.
* Data Collection Service is used for collecting data from the real world. It supports various real-world data collection technologies and strives to achieve the best compromise between data latency, data transmission bandwidth, data quality, and computing resources.
* Data Generation Service is used for generating data from simulation environments. It supports various simulation environment data generation technologies and can perform spatial, temporal, and unit alignment on the data.
* Dataset Construction Service is used for automated dataset construction. It can categorize model data, task data, scene data, and execution data into a pyramid structure and provide a qualitative assessment of the dataset's performance potential.

Note: The current version (v1.2) of open-source components includes Teleoperation Interface, Robot Interface，Sensor Interface and Data Collection Service.

<center> 
<img src="./src/airspeed.png" width="50%">
</center>

## 📂 Directory Structure and Module Description

- **src/** Main business code modules:
  - **teleoperation_interface/** Teleoperation service (VR/gamepad, etc. → standard ROS2 topics)
  - **robot_interface/** Unified robot API adaptation and data feedback
  - **sense_interface/** Sensors (currently focused on depth cameras, optional implementation)
  - **data_collection_service/** Efficient sampling and storage of system state, actions, and perception streams

Other directories:
- **install/** & **build/** colcon/ROS2 build artifacts
- **docs/** (if any) Supplementary documentation materials

---

## ⚡ Quick Start: Step-by-Step Execution and One-Click Launch

Recommended for first-time use (suggested: Ubuntu 22.04 + ROS2 Humble):

```bash
colcon build        # Initial build of all sub-packages
source install/setup.bash
```

Each main interface can be launched with one click:

### Teleoperation Interface
```bash
cd src/teleoperation_interface
./launch.sh
```

### Robot Interface
```bash
cd src/robot_interface
./launch.sh
```

### Sensor Interface (Optional)
```bash
cd src/sensor_interface
./launch.sh
```

### Data Collection/Storage
```bash
cd src/data_collection_service
./launch.sh
```

---

## 🪐 Typical Data Flow Interpretation

1. **Teleoperation device (e.g., VR) initiates commands → standardized ROS2 topics → robot_interface calls control APIs, enabling cross-brand control.**
2. **Robot interface continuously returns state, pose, etc.; all actions and states are automatically collected by data_collection_service and flexibly saved to disk based on configuration (button/automatic).**
3. **sense_interface supplements multimodal sensing (e.g., images/point clouds), and data is automatically archived.**
4. **Platform configuration is highly open, requiring only YAML/JSON configuration to flexibly adapt to different scenarios and hardware.**

---

## 🔧 Dependency Environment and Configuration

- **Recommended Environment**: Ubuntu 22.04, ROS 2 Humble, Python 3.10
- **Python Dependencies**: See sub-package README for requirements of each sub-package, such as aiohttp, numpy, pyyaml, h5py, pymycobot, etc.
- **ROS2 Dependencies**: `geometry_msgs`, `std_msgs` are included with ROS2
- **Hardware**: Supports common robotic arms (e.g., ElephantRobot, UR, Franka), various VR devices, and Realsense cameras, etc.
- **Configuration File Examples**: See config/ directories and README details in each sub-package

---

## 🏃 Typical Application Cases

- Using VR devices to remotely and precisely control robotic arms, generating high-quality training data for reinforcement/imitation learning
- Laboratory multi-robot hardware collaborative calibration, teaching, and experimental workflows
- Integrated closed-loop embodied AI model development with convenient team iteration and reproduction

---

## 🧐 Frequently Asked Questions (FAQ)

- **Q: One-click scripts won't run/missing dependencies?**
  - Please build once (colcon build) and source the environment script first
  - Each package script has automatic dependency checking/error reporting
- **Q: No topic data/robot not responding?**
  - Check node logs, verify data flow with `ros2 topic list`/`echo`
  - Please refer to each sub-package documentation for configuration file paths and parameters
- **Q: Can I change robots or extend new perception/devices?**
  - Yes, supported. Just organize configuration and API bindings. See robot_interface/README.md for details

---

## 📚 More Documentation and Support

Each main directory README.md, examples and configurations, and scripts provide detailed explanations. For extension development, bug reports, or kernel adaptation suggestions, please contact maintainers through project issues or email.

---

Wishing you success in achieving efficient robot teleoperation and closed-loop training on the Airspeed platform, creating outstanding embodied AI engineering innovations!

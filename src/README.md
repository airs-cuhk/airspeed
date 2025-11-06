# AIRSPEED 通用具身智能数据生产平台

Airspeed 是一个支持“任意遥操作设备—任意机器人—自动数据采集”的通用平台，通过强大的开源分层架构，助力机器人交互与具身智能模型快速闭环。

---

## ✨ 项目简介

Airspeed 提供了一套统一接口，将各种VR遥操作设备、异构机器人、本地/云端存储与感知设备无缝集成起来，实现：
- 遥操作设备“即插即用”
- 不同品牌/类型机器人灵活切换
- 全流程任务数据自动采集存储
- 实时数据驱动的智能体训练与闭环优化

---

## 🚀 快速上手（推荐一键启动全流程）

每个子模块均支持一键启动脚本，首次体验建议按如下顺序依次运行（在各自包目录下执行）：

### 1. 启动 teleoperation_interface（VR遥操作服务）
```bash
cd src/teleoperation_interface
./launch.sh
```
或
```bash
cd /home/airspeedbox/code/vr_robot_control_ws
ros2 launch teleoperation_interface vr_teleoperation.launch.py
```

### 2. 启动 robot_interface（机器人接口）
```bash
cd src/robot_interface
./launch.sh
```
或
```bash
cd /home/airspeedbox/code/vr_robot_control_ws
ros2 launch robot_interface robot_interface.launch.py
```

### 3. 启动 sensor_interface（可选，用于深度相机等感知设备）
```bash
cd src/sensor_interface
./launch.sh
```
或
```bash
cd /home/airspeedbox/code/vr_robot_control_ws
ros2 launch sensor_interface realsense_camera.launch.py
```

### 4. 启动 data_collection_service（数据采集/存储）
```bash
cd src/data_collection_service
./launch.sh
```
或
```bash
cd /home/airspeedbox/code/vr_robot_control_ws
ros2 launch data_collection_service data_storage.launch.py
```

> **注意**：所有运行均建议先 `colcon build` 构建一次并 `source install/setup.bash`。所有脚本支持调试、配置、环境检查。

---

## 📦 目录结构 & 各接口简介

- **teleoperation_interface/**  
  VR/远程遥操作入口。接收VR设备/手柄等多源数据，通过 HTTPS→ROS2 低延迟发布标准化控制话题（见 vr_teleoperation/vr_data_server.py、vr_teleoperation_node.py）。
  - 一键启动：`src/teleoperation_interface/launch.sh`
  - 主要话题：`/target_left/right_arm_pose` `/_buttons` `/_head_pose`

- **robot_interface/**  
  通用机器人适配层。接收遥操作的 ROS2 话题，转换并调用对应品牌机器人API。内建异步命令队列与安全状态机，支持机器人切换（以 config/robot_config.yaml 配置）。
  - 一键启动：`src/robot_interface/launch.sh`
  - 主要话题：`/robot_joint_angles` `/robot_cartesian_pose`

- **sensor_interface/**（可选）  
  机器人感知统一接口，目前主支持 RealSense 深度摄像头，自动发布图像、点云等多模态数据到 ROS2。
  - 一键启动：`src/sensor_interface/launch.sh`
  - 主要话题：`/camera/color/image_raw` `/camera/depth/image_raw` `/camera/depth/points`

- **data_collection_service/**  
  全流程采集与存储服务，自动订阅机器人各状态、遥操作信号、传感器输出。支持VR按钮/命令行灵活控制录制、自动多频率同步，数据直接落盘（HDF5/CSV/JSON/PKL）。
  - 一键启动：`src/data_collection_service/launch.sh`
  - 自动存储：`data/`、`data/vr_sessions/`

---

## 🔄 典型数据流（架构总览）

1. **VR遥操作设备（如Quest/Pico）通过HTTPS发送姿态数据 → teleoperation_interface → 转ROS2标准话题**
2. **robot_interface实时监听这些话题、完成数据转换与动作控制，下发到具体机器人**
3. **robot_interface将状态/位姿/关节等回传同步输出为状态话题**
4. **data_collection_service自动采集遥操作信号/机器人状态/传感感知流，并按配置保存为训练样本**
5. **（可选）sensor_interface统一管理与采集相机等感知设备数据供训练与回放**

---

## 🛠️ 典型应用场景

- 跨设备遥操作（VR设备、操纵杆等）实时驱动实际机械臂且采集全流程数据
- 任意机器人快速接入（只需适配API & 配置）
- 自动生成具身智能训练集，支撑下游决策或模仿学习模型
- 科研/教学实验，针对新硬件或新场景零代码迁移

---

## 📚 文档与开发支持

- 各子包下 README 与配置文件有详细说明，请优先参考（如 data_collection_service/README.md、robot_interface/README.md、teleoperation_interface/README.md、sensor_interface/README.md）
- 常见问题（FAQ）和调试命令：见各包 README
- 故障排查建议：优先关注启动日志、话题数据流（用 `ros2 topic list/echo` 排查）

---

## 📝 联系与贡献

如需自定义开发、Bug反馈或机器人/感知适配扩展建议，请参考各模块 README 开发手册，或通过邮件/issue 联系维护者。

---

祝您在 Airspeed 平台上快速实现机器人控制与模型训练闭环，开启高效具身智能创新之旅！




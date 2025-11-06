# Sensor Interface Package

Sensor Interface 是一个通用传感器接口ROS2节点包，用于发布Realsense D435i相机数据。

## 功能特性

- 支持Realsense D435i相机
- 发布RGB图像、深度图像和点云数据
- 支持相机参数配置
- 提供TF变换广播
- 与data_collection_service包集成

## 依赖安装

### 1. 安装Realsense SDK

```bash
# 注册服务器的公钥
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# 将服务器添加到存储库列表中
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 安装SDK
sudo apt-get install librealsense2-utils librealsense2-dev
```

### 2. 验证安装

```bash
# 打开Realsense查看器
realsense-viewer
```

### 3. 安装Python依赖

```bash
pip install pyrealsense2 opencv-python
```

# 检查设备
python -c "import pyrealsense2 as rs; ctx = rs.context(); print('Devices:', len(ctx.query_devices()))"

## 使用方法

### 1. 一键启动（推荐）

使用提供的shell脚本快速启动相机节点：

```bash
# 切换到工作空间根目录，运行一键启动脚本
cd /home/airspeedbox/code/vr_robot_control_ws
./src/sensor_interface/launch.sh
```

**脚本功能：**
- 自动检查工作空间环境
- 自动激活conda环境（如果存在）
- 自动设置ROS2环境
- 检查Realsense设备连接
- 验证Python依赖
- 启动相机节点

### 2. 手动启动相机节点

```bash
#先激活环境
conda activate ros2_env
cd /home/airspeedbox/code/vr_robot_control_ws
source install/setup.bash

# 启动相机节点
ros2 launch sensor_interface realsense_camera.launch.py

# 或使用自定义配置文件
ros2 launch sensor_interface realsense_camera.launch.py config_file:=/path/to/your/config.yaml
```

### 3. 启动完整系统

```bash
# 启动包含相机、机器人接口和数据存储的完整系统
ros2 launch data_collection_service complete_system.launch.py
```

## 发布的话题

- `/camera/color/image_raw` (sensor_msgs/Image) - RGB图像
- `/camera/depth/image_raw` (sensor_msgs/Image) - 深度图像
- `/camera/color/camera_info` (sensor_msgs/CameraInfo) - RGB相机信息
- `/camera/depth/camera_info` (sensor_msgs/CameraInfo) - 深度相机信息
- `/camera/depth/points` (sensor_msgs/PointCloud2) - 点云数据

## Shell脚本说明

### launch.sh

位于 `sensor_interface/launch.sh` 的一键启动脚本，提供以下功能：

**自动检查：**
- 工作空间环境是否正确
- conda环境是否存在并自动激活
- ROS2环境设置
- Realsense设备连接状态
- Python依赖包安装情况

**使用方法：**
```bash
# 在工作空间根目录下运行
./src/sensor_interface/launch.sh
```

**脚本特性：**
- 彩色输出，便于查看状态信息
- 错误处理和友好的错误提示
- 支持中断信号处理（Ctrl+C）
- 自动环境检测和设置

## 配置文件

配置文件位于 `config/realsense_config.yaml`，包含以下配置：

- 相机参数（分辨率、帧率等）
- 话题名称
- TF变换设置
- 性能配置

## 与数据存储集成

相机数据会自动被data_collection_service节点订阅并存储到HDF5文件中，包括：

- RGB图像数据
- 深度图像数据
- 点云元数据
- 相机参数信息

## 故障排除

### 1. 相机未检测到

```bash
# 检查USB连接
lsusb | grep Intel

# 检查相机权限
sudo chmod 666 /dev/video*
```

### 2. 依赖问题

```bash
# 重新安装依赖
sudo apt-get update
sudo apt-get install librealsense2-utils librealsense2-dev

# 检查Python包
pip list | grep pyrealsense2
```

### 3. 权限问题

```bash
# 添加用户到video组
sudo usermod -a -G video $USER
# 重新登录生效
```

## 性能优化

- 调整图像分辨率和帧率以平衡性能和质量
- 使用压缩存储减少存储空间
- 根据需要启用/禁用点云数据发布

## 技术支持

如有问题，请检查：

1. 相机硬件连接
2. 依赖库安装
3. 配置文件设置
4. ROS2节点状态




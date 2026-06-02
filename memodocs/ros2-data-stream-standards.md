# ROS2 机器人控制数据流标准

## 接口定义语言 (IDL) 底层类型标准

ROS2 的 `.msg` / `.srv` / `.action` 文件基于 **OMG IDL 4.2 子集** 定义，支持 **9 种基本类型**：

| 基本类型 | 说明 |
|----------|------|
| `bool` | 布尔值 |
| `byte` | 8位无符号整型 |
| `char` | 8位有符号整型 |
| `int8` / `uint8` | 8位整型 |
| `int16` / `uint16` | 16位整型 |
| `int32` / `uint32` | 32位整型 |
| `int64` / `uint64` | 64位整型 |
| `float32` / `float64` | 单/双精度浮点 |
| `string` | 字符串 |

- 数组：类型后加 `[]`（变长，C++映射为 `std::vector`）或定长 `T[N]`（映射为 `std::array<T,N>`）
- 支持结构体嵌套、常量定义（如 `int8 FOO=1`）
- `@key` 注解可标记主键字段用于 DDS 实例区分

---

## 通信模型的三层标准

| 模型 | 文件格式 | 通信模式 | 适用场景 |
|------|----------|----------|----------|
| **话题 (Topic)** | `.msg` | 发布-订阅（异步） | 高频数据流（传感器、状态、指令） |
| **服务 (Service)** | `.srv` | 请求-响应（同步） | 短任务（参数查询、触发动作） |
| **动作 (Action)** | `.action` | 目标→反馈→结果（异步有状态） | 长任务（导航、机械臂轨迹） |

### .msg 消息定义示例
```
# sensor_msgs/msg/LaserScan
std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 range_min
float32 range_max
float32[] ranges
```

### .srv 服务定义示例
```
# 请求
float32 distance
---
# 响应
float32 pose
```

### .action 动作定义示例
```
# 目标
float32 x
float32 y
---
# 结果
bool success
---
# 反馈
float32 progress
```

---

## 核心标准消息包 (common_interfaces)

### 1. `std_msgs` — 基础元数据

| 类型 | 用途 |
|------|------|
| `Header` | **核心元数据**：`stamp`（时间戳）+ `frame_id`（坐标系ID），几乎所有传感器消息都嵌套它 |
| `Bool` | 开关状态、急停信号 |
| `String` | 文本传输（日志、识别结果） |
| `Int32` / `Float64` | 数值传递（速度、计数器等） |
| `Empty` | 空消息，用作触发信号 |
| `ColorRGBA` | 颜色表示 |

### 2. `geometry_msgs` — 运动控制核心

| 类型 | 字段 | 用途 |
|------|------|------|
| `Point` | `x, y, z` | 3D 空间点 |
| `Vector3` | `x, y, z` | 3D 向量（力、速度） |
| `Quaternion` | `x, y, z, w` | 四元数姿态表示 |
| `Pose` | `position` + `orientation` | 位姿（位置+姿态） |
| `Twist` | `linear` + `angular` | **速度指令标准**，用于 `/cmd_vel` 话题 |
| `PoseStamped` | `header` + `pose` | 带时间戳和坐标系的位姿 |
| `Transform` | `translation` + `rotation` | 坐标变换 |
| `TransformStamped` | `header` + `child_frame_id` + `transform` | TF 广播的标准格式 |
| `PoseWithCovariance` | `pose` + 6×6 covariance | 带不确定性的位姿 |

### 3. `sensor_msgs` — 传感器数据

| 类型 | 内容 |
|------|------|
| `Imu` | 姿态四元数 + 角速度 + 线加速度 + 协方差矩阵 |
| `LaserScan` | 角度范围 + 距离数组 + 扫描时间 |
| `Image` | 编码格式 + 高宽 + 步长 + 原始像素数据 |
| `CameraInfo` | 相机内参 + 畸变系数 |
| `PointCloud2` | 点云数据 |
| `JointState` | 关节名列表 + 位置/速度/力数组 |
| `Range` | 超声波/红外距离传感器 |
| `BatteryState` | 电池状态（电压、电流、电量百分比） |
| `NavSatFix` | GPS 定位（纬度、经度、海拔） |

### 4. `nav_msgs` — 导航

| 类型 | 内容 |
|------|------|
| `Odometry` | 里程计（位姿 + 速度 + 6×6 协方差矩阵） |
| `Path` | 路径规划结果（`Header` + `PoseStamped[]`） |
| `OccupancyGrid` | 占据栅格地图（SLAM 输出） |

### 5. `trajectory_msgs` — 轨迹控制

| 类型 | 内容 |
|------|------|
| `JointTrajectory` | 关节空间轨迹（多关节+多点时间序列） |
| `JointTrajectoryPoint` | 单轨迹点（位置/速度/加速度/力 + 时间） |
| `MultiDOFJointTrajectory` | 多自由度末端轨迹 |

### 6. 其他常用包

- `action_msgs` — 动作通信底层协议
- `diagnostic_msgs` — 系统诊断（硬件健康度）
- `tf2_msgs` — 坐标变换广播（`TFMessage`）
- `visualization_msgs` — RViz 可视化标记（`Marker`）
- `shape_msgs` — 碰撞检测用几何形状

---

## ros2_control 硬件接口标准

在 `ros2_control` 框架中，硬件抽象层接口命名遵循 **`<joint_name>/<interface_type>`** 约定。

### Command Interfaces（控制指令）
```
<joint_name>/position
<joint_name>/velocity
<joint_name>/effort
<joint_name>/pid_gain
```

### State Interfaces（状态反馈）
```
<joint_name>/position
<joint_name>/velocity
<joint_name>/effort
<sensor_name>/force.x
<sensor_name>/force.y
<sensor_name>/force.z
<sensor_name>/torque.x
<sensor_name>/torque.y
<sensor_name>/torque.z
```

---

## 典型数据流频率参考

| 话题 | 消息类型 | 典型频率 | 用途 |
|------|----------|----------|------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ~100 Hz | 运动速度指令 |
| `/joint_states` | `sensor_msgs/msg/JointState` | ~500 Hz | 关节状态反馈 |
| `/odom` | `nav_msgs/msg/Odometry` | ~50 Hz | 里程计 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态 | 坐标变换树 |
| `/imu` | `sensor_msgs/msg/Imu` | ~200 Hz | IMU 惯性数据 |
| `/scan` | `sensor_msgs/msg/LaserScan` | ~10-40 Hz | 激光雷达扫描 |
| `/lowcmd` | 自定义 | ~500 Hz | 底层电机直控（如 Unitree） |

---

## 设计原则

1. **必须使用 `std_msgs/Header`**：任何涉及时间和坐标系的数据都必须嵌套 Header（`stamp` + `frame_id`），这是 ROS 多传感器融合和 TF 坐标变换的基础
2. **优先复用标准消息**：除非特殊硬件需求，优先使用 `common_interfaces`，确保与 Nav2、MoveIt2 等开源栈无缝集成
3. **IDL 自动代码生成**：`.msg` 文件通过 `rosidl` 工具链自动生成 C++/Python 代码，保证类型安全
4. **DDS 底层传输**：基于 DDS 的 QoS（可靠性、持久性、截止时间等）策略可精细控制每条数据流的行为
5. **语义化版本**：新增字段时保证向后兼容

---

## 序列化与传输

- 底层基于 **OMG IDL + DDS (Data Distribution Service)** 进行序列化和通信
- 支持 Protobuf 和原生 IDL 两种序列化方案
- 在 Python 中，数值数组可映射为 `numpy.ndarray` 提升性能

## 实用命令

```bash
# 查看消息定义
ros2 interface show sensor_msgs/msg/Image
ros2 interface show std_msgs/msg/Header
ros2 interface show geometry_msgs/msg/Pose

# 列出所有可用接口
ros2 interface list

# 列出 ros2_control 硬件接口
ros2 control list_hardware_interfaces

# 列出硬件组件
ros2 control list_hardware_components
```

---

## 项目合规审计：`vr-ik-robot-data-collection`

审计日期：2026-05-27

审计范围：
- `nodes/ik_split_publisher.py`
- `nodes/robot_state_ros2_publisher.py`
- `launch/start_all.sh`、`launch/start_vr.sh`
- `config/session_vr_ik_robot.yaml`、`config/session_vr_ik_robot_button_control.yaml`
- `camera-tests/robosense_data_stream.py`、`camera-tests/robosense_full_stream.py`
- `scripts/arm_calibrate.py`

---

### 合规项

| # | 项目 | 详情 |
|---|------|------|
| 1 | VR 位姿消息类型 | `/vr/head_pose`、`/vr/left_pose`、`/vr/right_pose` 使用 `geometry_msgs/PoseStamped` |
| 2 | 相机消息类型 | `/camera/*` 使用 `sensor_msgs/Image`，JPEG 编码 |
| 3 | 深度/红外消息类型 | `/robosense/depth/image_raw` 使用 `16UC1`，`/robosense/infra_*` 使用 `mono8` |
| 4 | 相机 Header 填充 | `robot_state_ros2_publisher.py:247-253` 和 `robosense_full_stream.py:274-279` 正确设置 `header.stamp` 和 `header.frame_id` |
| 5 | 话题命名 | 全部使用小写+下划线，命名空间化（`/vr/`、`/ik/`、`/arm/`、`/camera/`、`/robosense/`） |
| 6 | 关节单位 | IK 指令和手臂关节数据使用弧度 |
| 7 | QoS 配置 | `BEST_EFFORT` + `VOLATILE` + `KEEP_LAST` + `depth=1` 适用于高频遥测 |
| 8 | rclpy 生命周期 | 所有节点正确遵循 `init()` → `create_publisher` → `spin()` → `destroy_node()` → `shutdown()` |
| 9 | RoboSense 硬件时间戳 | `robosense_full_stream.py` 使用 `_rs_ts(frame)` 获取硬件时间戳 |

---

### 偏差项

####  CRITICAL-1：关节数据使用 `Float32MultiArray` 而非 `sensor_msgs/JointState`

影响 **8 个话题**：

| 话题 | 当前类型 | 应使用类型 |
|------|-----------|------------|
| `/ik/left_arm` | `Float32MultiArray` | `JointState` |
| `/ik/right_arm` | `Float32MultiArray` | `JointState` |
| `/ik/left_gripper` | `Float32MultiArray` | `JointState` |
| `/ik/right_gripper` | `Float32MultiArray` | `JointState` |
| `/arm/left_joints` | `Float32MultiArray` | `JointState` |
| `/arm/right_joints` | `Float32MultiArray` | `JointState` |
| `/arm/left_gripper` | `Float32MultiArray` | `JointState` |
| `/arm/right_gripper` | `Float32MultiArray` | `JointState` |

**`sensor_msgs/JointState` 提供而 `Float32MultiArray` 缺失的内容：**

- `Header header` — 采集时间戳 + 坐标系
- `string[] name` — 关节名称，消费者据此可知哪个值对应哪个关节
- `float64[] velocity` — 关节速度（当前完全缺失）
- `float64[] effort` — 关节力/力矩（当前完全缺失）
- 精度：`float64` 对比当前使用的 `float32`

**影响：** 任何标准 ROS2 工具（rviz2、plotjuggler、joint_state_broadcaster）消费这些话题时都需要自定义适配代码。

**相关代码位置：**
- `robot_state_ros2_publisher.py:169-176` — 创建 Float32MultiArray 发布者
- `robot_state_ros2_publisher.py:214-217` — `_publish_array()` 调用
- `robot_state_ros2_publisher.py:264-267` — `_publish_array()` 定义：纯数据，无 header，无关节名称
- `ik_split_publisher.py:63-70` — 创建 Float32MultiArray 发布者
- `ik_split_publisher.py:89-106` — 拆分逻辑：仅复制 float 值

####  CRITICAL-2：关节/夹爪/按钮数据无 Header

`robot_state_ros2_publisher.py:264-267`：
```python
@staticmethod
def _publish_array(pub, data: list) -> None:
    msg = Float32MultiArray()
    msg.data = [float(v) for v in data]
    pub.publish(msg)
```
无 `stamp`，无 `frame_id`。

`ik_split_publisher.py:89-90`：
```python
left_arm_msg = Float32MultiArray()
left_arm_msg.data = [float(v) for v in data[LEFT_ARM_START:LEFT_ARM_END]]
```
同样缺少。

**变通方案：** YAML 配置中使用 `time_domain: ros_receive`，以消息*到达*数据收集服务的时间作为替代。这会将网络延迟混入时间戳。标准做法是使用 `time_domain: ros_header`，由发布者在消息中嵌入采集时间。

####  中等-1：夹爪单位不一致

- `ik_split_publisher.py`：夹爪值以**弧度**为单位（来自 IK 求解器，原始值直接传递）
- `robot_state_ros2_publisher.py:105-106`：夹爪值通过 `np.rad2deg()` 转换为**度**

`/ik/left_gripper` 和 `/arm/left_gripper` 名义上是同一物理量，但单位不同。任何对比或组合这两路数据的消费者都会收到错误结果。

####  中等-2：`robot_state_ros2_publisher.py` 中相机时间戳使用 `get_clock().now()` 而非采集时间

`robot_state_ros2_publisher.py:247`：
```python
img_msg.header.stamp = self.get_clock().now().to_msg()
```
记录的是 JPEG 编码发生的时刻，而非相机曝光时刻。

对比 `robosense_full_stream.py:274`，后者通过 `_rs_ts(frame)` 正确使用了硬件时间戳。

####  轻微-1：`Float32MultiArray` 用于单元素夹爪值

发布 `Float32MultiArray` 且 `data=[1.23]` 对于单个标量来说结构过度。更合适的备选方案：
- `std_msgs/Float32` — 如果仅需位置
- `sensor_msgs/JointState` — 与关节数据结合时（推荐方案）

####  轻微-2：节点命名与核心 ROS2 包冲突

`robot_state_ros2_publisher.py:163`：
```python
super().__init__("robot_state_publisher")
```
与发布 TF 坐标帧的 `robot_state_publisher` 标准包重名。

####  轻微-3：IK 分割输入使用 `TRANSIENT_LOCAL`

`ik_split_publisher.py:29`：
```python
_INPUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 对指令流不标准
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
```
`TRANSIENT_LOCAL` 通常用于配置类数据，使晚期加入的订阅者能收到最后一条已知消息。对于持续高频指令流，`VOLATILE` 更为标准。

####  轻微-4：未使用 Service 或 Action

所有通信均使用 Topic。校准（`arm_calibrate.py`）直接调用 Python API 而不使用 ROS2 Service。在基于 ROS2 的系统中，校准、归零、急停等操作自然适合 Service/Action 模式。

####  轻微-5：`float32` 精度损失

`Float32MultiArray` 使用 `float32`（约 7 位有效数字），而 `JointState` 使用 `float64`（约 15 位有效数字）。对于以弧度为单位且精度要求 < 0.001° 的关节位置，float32 精度在极端值下可能不足。

---

### 修复优先级

| 优先级 | 变更 | 影响文件 |
|------|------|----------|
| **P0** | 将 8 个关节/夹爪话题切换为 `sensor_msgs/JointState` | `robot_state_ros2_publisher.py`、`ik_split_publisher.py`、两个 YAML 配置文件 |
| **P0** | 在 `JointState.header.stamp` 中添加采集时间戳，将 YAML 中的 `time_domain` 切换为 `ros_header` | 同上 + YAML |
| **P1** | 统一夹爪单位：全部使用弧度（移除 `np.rad2deg`） | `robot_state_ros2_publisher.py:105-106` |
| **P1** | 使用相机硬件时间戳（从帧元数据提取，或使用 `_rs_ts` 等效方式） | `robot_state_ros2_publisher.py:247` |
| **P2** | 将节点重命名为 `arm_state_publisher` 或 `openarm_state_publisher` | `robot_state_ros2_publisher.py:163` |
| **P2** | 将 IK 分割输入 QoS 切换为 `VOLATILE` | `ik_split_publisher.py:27-32` |
| **P3** | 为校准添加 ROS2 Service | 新建 `srv/CalibrateArm.srv`、修改 `arm_calibrate.py` |
| **P3** | 添加 `JointState.name` 以标识 `["left_joint_1", ..., "left_joint_7"]` 等 | `robot_state_ros2_publisher.py`、`ik_split_publisher.py` |

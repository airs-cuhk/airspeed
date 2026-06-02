# 机器人接口管道映射 — 调试参考

日期: 2026-06-02

完整管道(IK): VR ROS2 → 归一化 → IK求解 → ROS2 JointState + WS手臂广播
完整管道(控制): CAN总线 → JointState发布

---

## IK管道

### HOP 0: VR订阅器

**维度空间**: ROS2 DDS → 线程安全数据存储

```
vr_subscriber.py:149-175 (class VRSubscriber)
```
```python
class VRSubscriber:
    def __init__(self, config: VRConfig) -> None:
        self.config = config
        self.data_store = VRDataStore(stale_timeout_s=config.calibration.stale_timeout_s)
        self._thread: threading.Thread | None = None
        self._ros2_available = False

    def start(self) -> bool:
        try:
            import rclpy
            self._ros2_available = True
        except ImportError:
            return False
        self._thread = threading.Thread(target=self._run_ros2, daemon=True,
                                        name="vr-ros2-spin")
        self._thread.start()
        return True
```

**公式**: 无 (结构性的——ROS2订阅 + 线程安全存储)

**说明**: 在守护线程中订阅5个VR桥话题(`/vr/head_pose`, `/vr/left_pose`, `/vr/right_pose`, `/vr/left_buttons`, `/vr/right_buttons`)。将最新消息存储在带锁的线程安全`VRDataStore`中。提供`get_head_pose()`, `get_left_pose()`等供求解器循环读取, 无需ROS2耦合。参考来源: `config/vr.yaml` `ros2.topics`。

### 功能描述

在后台线程中订阅VR桥ROS2话题。将最新姿态和按钮数据存储在线程安全字典中。求解器循环读取此存储——不直接接触ROS2。

---

### HOP 1: VR归一化

**维度空间**: VR原生坐标 (Y-up, 右手系) → 机器人基坐标系 (Z-up)

```
solver_loop.py:66-77
```
```python
if vr_normalizer is not None and vr_subscriber is not None:
    vr_result = vr_normalizer.normalize(vr_subscriber.data_store)
    if vr_result.state.value == "active":
        target_L = vr_result.left
        target_R = vr_result.right
        left_trigger = vr_result.left_trigger
        right_trigger = vr_result.right_trigger
```

**公式**: `T_robot = (T_origin)^(-1) * T_VR * M_axis` 其中`M_axis`是来自`config/vr.yaml`的3×3轴映射矩阵, `T_origin`是固定的标定原点。

**说明**: VR归一化器应用两级变换: (1)将VR控制器位姿重定位到固定的VR原点坐标系(操作员按B按钮设定), (2)应用从原生VR坐标到机器人基坐标系的轴映射矩阵。归一化器有三种状态: `waiting`(无原点), `ready`(原点已固定但未追踪), `active`(产生SE3目标)。扳机值从按钮数据提取并作为夹爪控制值传递。参考来源: `config/vr.yaml` `axis_mapping`和`calibration`节。

### 功能描述

使用固定原点和轴映射矩阵将VR控制器姿态从原生坐标变换到机器人基坐标系。仅在`active`标定状态下产生SE3目标。

---

### HOP 2: IK求解

**维度空间**: SE3末端执行器目标 → 关节角度 (弧度)

```
solver_loop.py:91-93
```
```python
if target_L is not None or target_R is not None:
    result = await ik_service.solve_targets(target_L, target_R)
```

**公式**: `q* = argmin_q Σi wi * ci(q, TL, TR)` 其中代价包括: 位置误差(50×), 朝向误差(10×), 静止姿态(5×), 关节限制(100×), 自碰撞(10×), 可操作度。权重来自`config/solver_smooth.yaml`。

**说明**: 基于JAX的非线性优化, 使用PyrokiSolver。从前次解热启动(响应式)或静止姿态(确定性, 若`use_home_warm_start: true`)。在线程池执行器中运行以避免阻塞异步事件循环。每臂产生9个关节(7自由度 + 2个手指关节)。求解速率: 目标50 Hz, 空闲时约40 Hz, 求解时降低。参考来源: `config/solver_smooth.yaml`和`openarm_ik_solver/solver.py`。

### 功能描述

使用基于JAX的非线性优化求解双臂逆运动学。输入左右SE3目标, 返回每臂9个关节角度(弧度)。从前次解热启动以提高响应速度。以50 Hz运行。

---

### HOP 3: 夹爪覆盖

**维度空间**: 关节角度 → 关节角度 (手指关节由扳机覆盖)

```
solver_loop.py:119-128
```
```python
if result is not None and result.success and len(result.joint_radians_left) == 9:
    result.joint_radians_left[7] = 0.044 * (1.0 - left_trigger)
    result.joint_radians_left[8] = 0.044 * (1.0 - left_trigger)
    result.joint_radians_right[7] = 0.044 * (1.0 - right_trigger)
    result.joint_radians_right[8] = 0.044 * (1.0 - right_trigger)
```

**公式**: `q_finger = 0.044 * (1.0 - τ)` 其中 `τ ∈ [0, 1]` 为扳机值
完全按下`(τ=1)` → `q=0.0`(闭合)。完全释放`(τ=0)` → `q=0.044`(张开)。

**说明**: IK求解器不优化手指关节——它们直接由VR扳机值设定。每只手两个手指关节(索引7和8)获得相同值。常量`0.044`(米)为URDF手指关节范围。扳机读取自VR按钮数据通道0(模拟扳机)。参考来源: URDF关节限制`openarm_left_finger_joint1` = 0.044m。

### 功能描述

用VR扳机值覆盖手指关节角度。扳机=0(释放)将手指设为0.044m(张开)。扳机=1(按下)将手指设为0.0m(闭合)。每只手两个手指获得相同值。

### 输出示例

**输入**: `left_trigger = 0.5`, `right_trigger = 0.8`
**输出**:
```
joint_radians_left[7]  = 0.022   ← 0.044 * (1.0 - 0.5), 半闭合
joint_radians_left[8]  = 0.022   ← 与[7]相同
joint_radians_right[7] = 0.0088  ← 0.044 * (1.0 - 0.8), 大部闭合
joint_radians_right[8] = 0.0088  ← 与[7]相同
```

---

### HOP 4: 关节展平与分离

**维度空间**: 2×9关节数组 → 左8 + 右8数组

```
solver_loop.py:154-162
```
```python
ros2_publisher.publish(
    left_joints=result.joint_radians_left[:8],
    right_joints=result.joint_radians_right[:8],
    left_target_xyz=...,
    left_target_quat_xyzw=...,
    ...
)
```

**公式**: `L_out = [L_1..7, L_f1]` (8个值), `R_out = [R_1..7, R_f1]` (8个值)
第二个手指关节(索引8)被丢弃——始终等于索引7。

**说明**: 每臂9关节数组修剪为8: 7个手臂关节 + 1个手指关节。第二个手指关节(索引8, 始终等于索引7)被丢弃。四元数从WXYZ(求解器输出)转换为XYZW(ROS2 PoseStamped约定)。参考来源: `ros2_publisher.py` `publish()`签名和`sensor_msgs/JointState`消息定义。

### 功能描述

将18关节数组分离为每臂8个值。丢弃冗余的第二个手指关节。将目标四元数从WXYZ转换为XYZW以适配ROS2。

---

### HOP 5: ROS2 JointState发布

**维度空间**: Python数组 → `sensor_msgs/JointState` ROS2消息

```
ros2_publisher.py:156-161
```
```python
def _publish_joint_state(self, pub, stamp, frame_id, names, values):
    msg = JointState()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.name = names
    msg.position = [float(v) for v in values]
    pub.publish(msg)
```

**公式**: `msg.position = [v_1, ..., v_8] ∈ R^8` (弧度)
`msg.name = [openarm_left_joint1, ..., openarm_left_finger_joint1]`
QoS: BEST_EFFORT, VOLATILE, KEEP_LAST depth=1

**说明**: 每臂8个关节值发布到单独的`JointState`话题, 带URDF关节名称。手臂和夹爪关节合并在一条消息中(不同于旧的分立Float32MultiArray方式)。关节命令的QoS为BEST_EFFORT——丢失消息可接受。参考来源: `robot_interface/README.md` JointState约定。

### 功能描述

将每臂8关节(7手臂 + 夹爪)的`JointState`消息发布到ROS2话题。关节名称来自URDF模型。同时将目标姿态发布为`PoseStamped`。

---

### HOP 6: WebSocket手臂广播

**维度空间**: 关节数组 → JSON WebSocket消息

```
solver_loop.py (original):184-196
```
```python
data = json.dumps({
    "type": "arm_commands",
    "joints": joint_commands,        # 18 浮点数
    "left": joint_commands[:9],      # 左9个
    "right": joint_commands[9:],     # 右9个
    "left_gripper_deg": left_gripper_deg,
    "right_gripper_deg": right_gripper_deg,
})
for ws in list(arm_websockets):
    await ws.send_str(data)
```

**公式**: `msg = {type: "arm_commands", joints: [v_0..v_17], left: [v_0..v_8], right: [v_9..v_17], left_gripper_deg: γ_L, right_gripper_deg: γ_R}` 其中 `γ = -65.0 * (1.0 - τ)` (度, `τ`为扳机值)

**说明**: 完整的18浮点关节数组(包括两个手指关节)以JSON格式发送至所有连接的手臂WebSocket客户端。夹爪值以度为单位, 供电机控制器使用。这是控制路径——`/ws/arm`处的arm_controller根据此数据驱动物理电机。参考来源: arm controller `arm_controller.py`——期望`left`, `right`, `left_gripper_deg`, `right_gripper_deg`字段。

### 功能描述

将完整的18浮点关节命令(每臂9个, 含两个手指关节)和夹爪值(度)以JSON广播至`/ws/arm`的所有WebSocket客户端。这是电机控制路径——arm_controller根据此数据驱动电机。

---

## 控制管道

### HOP C0: CAN总线读取

**维度空间**: CAN总线 → 关节位置 (弧度)

```
arm_state_publisher.py:56-62
```
```python
def _read_arm_state(follower):
    obs = follower.get_observation()
    left = [float(obs.get(f"left_joint_{i}.pos", 0.0)) for i in range(1, 8)]
    right = [float(obs.get(f"right_joint_{i}.pos", 0.0)) for i in range(1, 8)]
    left_grip = float(obs.get("left_gripper.pos", 0.0))
    right_grip = float(obs.get("right_gripper.pos", 0.0))
    return left, right, left_grip, right_grip
```

**公式**: `q_left[i-1] = obs[left_joint_i.pos], q_right[i-1] = obs[right_joint_i.pos], i ∈ [1, 7]`
夹爪以弧度从CAN总线读取。返回`(7关节, 7关节, 夹爪_度, 夹爪_度)`。

**说明**: 通过lerobot OpenArmsFollower从CAN总线读取电机位置。follower直接连接CAN总线(无`calibrate()`, 无`enable_torque()`)——纯被动读取。夹爪位置以弧度读取并转换为度。参考来源: OpenArm电机CAN协议——每臂`joint_1`至`joint_7` + `gripper`。

### 功能描述

以纯只读模式从CAN总线读取每臂7个关节位置和夹爪位置。无扭矩命令, 无标定——仅被动观察。

---

### HOP C1: JointState发布

**维度空间**: 原始关节弧度 → `sensor_msgs/JointState` ROS2消息

```
arm_state_publisher.py:92-97
```
```python
lm = JointState()
lm.header.stamp = now
lm.header.frame_id = "base_link"
lm.name = self._left_names
lm.position = [float(v) for v in left[:7]] + [float(left_grip)]
self._left_pub.publish(lm)
```

**公式**: `msg.position = [q_1, ..., q_7, θ_gripper] ∈ R^8_rad`
话题: `/arm/left/joint_state`, `/arm/right/joint_state`

**说明**: 手臂状态(非命令)——发布机器人实际测量的关节位置。这是数据采集服务记录为机器人真值状态的数据。区别于`/arm/left/joint_commands`(IK求解器的期望位置)。参考来源: `robot_interface/README.md` JointState约定。

### 功能描述

将机器人CAN总线的实际测量关节位置发布为`JointState`话题。这是机器人的真值反馈——与IK求解器的命令话题不同。

---

## 总结: 机器人接口

### IK管道:
```
VR ROS2话题                              ROS2 DDS, 60 Hz
    |
    v HOP 0: VRSubscriber               线程安全数据存储
    |
    v HOP 1: VRNormalizer               机器人基坐标系中的SE3目标
    |
    v HOP 2: ik_service.solve_targets() 关节角度 (每臂9个, 弧度)
    |
    v HOP 3: 夹爪覆盖                    手指关节 = f(扳机)
    |
    v HOP 4: 展平 + 分离                 左8 + 右8 + 目标姿态
    |
    +--> HOP 5: ROS2发布                 /arm/left/joint_commands (JointState)
    |                                     /arm/left/target_pose (PoseStamped)
    |
    +--> HOP 6: WS广播                   /ws/arm → arm_controller → 电机
```

### 控制管道:
```
CAN总线                                  原始电机位置 (弧度)
    |
    v HOP C0: _read_arm_state()          每臂7关节 + 夹爪
    |
    v HOP C1: JointState发布              /arm/left/joint_state (JointState)
                                          /arm/right/joint_state (JointState)
```

### 关键参考文档:
1. **config/vr.yaml**: 轴映射矩阵, 标定状态机
2. **config/solver_smooth.yaml**: IK代价权重, 最大迭代次数, 碰撞余量
3. **config/robot.yaml**: URDF路径, 关节名称, 静止位置
4. **robot_interface/README.md**: JointState + PoseStamped约定
5. **openarm_ik_solver/solver.py**: PyrokiSolver——JAX非线性优化

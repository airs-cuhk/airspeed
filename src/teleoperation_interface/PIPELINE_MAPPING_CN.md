# VR桥接器管道映射 —调试参考

日期: 2026-06-02

完整管道: VR设备 HTTPS POST → JSON解析 → 字段提取 → ROS2 PoseStamped + Float32MultiArray 发布

---

## HOP 0: HTTPS接收

**维度空间**: HTTPS上的原始JSON文本

```
vr_bridge_server.py:291-292
```
```python
async def handle_pose(request: web.Request) -> web.Response:
    raw_text = await request.text()
```

**公式**: `raw = await(HTTP POST body) ∈ UTF-8字符串`

**说明**: aiohttp服务器接收来自VR设备浏览器的`/poseData` POST请求。VR网页界面(WebXR)以配置的频率(~60 Hz)发送姿态数据JSON。此阶段不进行验证——原始文本直接传递。参考来源: `main()`中第407行注册的`handle_pose`。

### 功能描述

接收HTTPS POST请求体为UTF-8字符串。VR设备浏览器以~60 Hz频率发送JSON姿态数据。不进行解析或验证——原始文本原样传递至下一HOP。

### 输出示例

**输入**: HTTPS POST 到 `/poseData`
**输出**: `raw_text = '{"state":"NORMAL","head":{"position":{"x":0.18,"y":1.33,"z":0.12},"rotation":{"x":...}},...}'`

关键特征:
- VR追踪时数据到达频率约60 Hz
- 字符串长度约500-1500字节，取决于正在追踪的身体部位
- JSON结构: `{state, battery, head?, left?, right?}` — 身体部位未被追踪时可能缺失

---

## HOP 1: JSON解析

**维度空间**: UTF-8字符串 → Python字典

```
vr_bridge_server.py:294-298
```
```python
    if _ros_node is not None:
        try:
            data = json.loads(raw_text)
        except json.JSONDecodeError:
            return web.Response(status=400, text="Invalid JSON")
```

**公式**: `D = json.loads(raw) ∈ {state: str, battery: float, head?: dict, left?: dict, right?: dict}`

**说明**: 将原始文本解析为Python字典。如果解析失败，返回HTTP 400。`_ros_node`检查控制整个发布路径——如果ROS2不可用，数据将被静默确认但不发布。参考来源: VR网页界面WebXR数据采集器定义的JSON结构。

### 功能描述

将原始JSON文本解析为Python字典。无效JSON返回HTTP 400。ROS2不可用时(`_ros_node is None`)静默跳过。

### 输出示例

**输入**: `'{"state":"NORMAL","head":{"position":{"x":0.18,"y":1.33,"z":0.12},"rotation":{"x":0.3,"y":-0.28,"z":0.18,"w":0.89}},...}'`
**输出**:
```python
data = {
    "state": "NORMAL",
    "battery": 0.8,
    "head": {
        "position": {"x": 0.18, "y": 1.33, "z": 0.12},
        "rotation": {"x": 0.3, "y": -0.28, "z": 0.18, "w": 0.89}
    },
    "left": {
        "position": {"x": -0.25, "y": 0.76, "z": -0.34},
        "rotation": {"x": -0.04, "y": 0.58, "z": -0.35, "w": 0.74},
        "button": [{"value": 0.0}, {"value": 0.0}, ...]
    },
    "right": {...}
}
```

关键特征:
- `head`、`left`、`right`键可能缺失 (设备未追踪)
- `button`数组包含带`value`键(0.0-1.0)的字典
- 所有位置值单位为米，旋转为归一化四元数(xyzw)

---

## HOP 2: 姿态字段提取

**维度空间**: Python字典 → `geometry_msgs/PoseStamped` (ROS2消息)

```
vr_bridge_server.py:256-269
```
```python
def _make_pose(data: Dict, stamp, frame_id: str) -> PoseStamped:
    m = PoseStamped()
    m.header.stamp = stamp
    m.header.frame_id = frame_id
    pos = data.get('position', {})
    m.pose.position.x = float(pos.get('x', 0.0))
    m.pose.position.y = float(pos.get('y', 0.0))
    m.pose.position.z = float(pos.get('z', 0.0))
    rot = data.get('rotation', {})
    m.pose.orientation.x = float(rot.get('x', 0.0))
    m.pose.orientation.y = float(rot.get('y', 0.0))
    m.pose.orientation.z = float(rot.get('z', 0.0))
    m.pose.orientation.w = float(rot.get('w', 1.0))
    return m
```

**公式**: `m.pose.position = data中的(x, y, z)` 单位为米
`m.pose.orientation = data中的(x, y, z, w)` 为归一化四元数
`m.header.stamp = t_receive` (桥接器接收时间, 非设备采集时间)

**说明**: 从每个身体部位字典中提取位置(x, y, z)和旋转(x, y, z, w四元数)。缺失字段默认为0.0。缺失`w`默认为1.0(单位朝向)。时间戳使用桥接器接收时的ROS2节点时钟——VR设备不嵌入采集时间戳。参考来源: WebXR `XRPose`规范——位置单位为米, 朝向为xyzw四元数。

### 功能描述

从身体部位字典构造`PoseStamped`消息。提取位置(米)和朝向(xyzw四元数)。使用桥接器接收时间作为时间戳——非硬件采集时间。分别应用于`head`、`left`和`right`字典(若存在)。

### 输出示例

**输入**: `data = {"position": {"x": 0.20, "y": 0.83, "z": -0.36}, "rotation": {"x": 0.23, "y": 0.16, "z": 0.35, "w": 0.89}}`, `stamp = Time(sec=..., nanosec=...)`, `frame_id = "vr_right"`
**输出**:
```
header:
  stamp: {sec: 1779954107, nanosec: 356294491}   ← 桥接器接收时间
  frame_id: vr_right                              ← 按身体部位标识
pose:
  position: {x: 0.20, y: 0.83, z: -0.36}         ← float64, 米
  orientation: {x: 0.23, y: 0.16, z: 0.35, w: 0.89}  ← 归一化四元数
```

关键特征:
- 时间戳非硬件采集时间——会话YAML使用`time_domain: ros_header`但需知其为桥接器接收时间
- `w`缺失时默认为1.0(单位四元数)——四元数模长保持不变
- 所有字段转换为Python float → ROS2 float64

---

## HOP 3: 按钮提取

**维度空间**: Python字典 → `std_msgs/Float32MultiArray` (ROS2消息)

```
vr_bridge_server.py:272-278
```
```python
def _make_buttons(buttons: List) -> Float32MultiArray:
    m = Float32MultiArray()
    vals = [float(b.get('value', 0)) for b in buttons[:6]]
    while len(vals) < 6:
        vals.append(0.0)
    m.data = vals
    return m
```

**公式**: `v_i = button_i.value, i ∈ [0, 5]`, 缺失按钮补零至0.0
`m.data = [v_0, ..., v_5] ∈ R^6[0.0, 1.0]`

**说明**: 从按钮数组中提取最多6个按钮值。每个按钮是一个`{"value": float}`字典。不足6个补零至0.0。按钮数组顺序由设备定义——参见teleoperation_interface README中的通道映射约定。参考来源: `teleoperation_interface/README.md`中设备特定的按钮映射。

### 功能描述

从`{"value": float}`字典列表中提取按钮值, 补零至6个通道, 封装为`Float32MultiArray`。无header(消息类型限制)——会话YAML中使用`time_domain: ros_receive`。

### 输出示例

**输入**: `buttons = [{"value": 0.0}, {"value": 0.0}, {"value": 1.0}, {"value": 0.0}, {"value": 0.0}, {"value": 0.5}]`
**输出**:
```
data: [0.0, 0.0, 1.0, 0.0, 0.0, 0.5]   ← 始终恰好6个浮点数
     ↑ 扳机  ↑ 握把   ↑ 主按钮              ← 通道含义: 设备定义
```

关键特征:
- 始终恰好6个通道——缺失按钮补零至0.0
- 无header.stamp——会话YAML必须使用`time_domain: ros_receive`
- 通道含义由设备定义——需文档化映射

---

## HOP 4: DDS发布

**维度空间**: ROS2消息对象 → DDS传输

```
vr_bridge_server.py:228-233
```
```python
    if 'head' in data:
        self.head_pub.publish(_make_pose(data['head'], now, 'vr_head'))
    if 'left' in data:
        self.left_pub.publish(_make_pose(data['left'], now, 'vr_left'))
        if 'button' in data['left']:
            self.left_btn_pub.publish(_make_buttons(data['left']['button']))
    if 'right' in data:
        self.right_pub.publish(_make_pose(data['right'], now, 'vr_right'))
        if 'button' in data['right']:
            self.right_btn_pub.publish(_make_buttons(data['right']['button']))
```

**公式**: 无 (结构性的——按身体部位条件发布)

**说明**: 将构造的ROS2消息发布至DDS。每个身体部位仅在其JSON数据存在时发布——如果VR设备未追踪某个控制器, 该话题保持静默。QoS: BEST_EFFORT, VOLATILE, KEEP_LAST, depth=1 (定义于第70行)。这意味着仅保留最新消息; 丢失的消息不会被重传。参考来源: `vr_bridge_server.py:70-77`中的QoS常量。

### 功能描述

对头显、左右控制器发布`PoseStamped`(若JSON中存在相应数据), 对控制器按钮发布`Float32MultiArray`。QoS为BEST_EFFORT/VOLATILE/KEEP_LAST depth=1——仅保留最新帧。

### 输出示例

**输入**: `_make_pose(data['right'], now, 'vr_right')`
**发布话题**:
```
/vr_raw_data        (String)          ← 原始JSON透传
/vr/head_pose       (PoseStamped)     ← 如果head存在
/vr/left_pose       (PoseStamped)     ← 如果left存在
/vr/right_pose      (PoseStamped)     ← 如果right存在
/vr/left_buttons    (Float32MultiArray) ← 如果left.button存在
/vr/right_buttons   (Float32MultiArray) ← 如果right.button存在
```

关键特征:
- 每个话题独立发布——缺失的身体部位不产生消息
- 总速率约60 Hz, 实测62 Hz (来自create_timer的轻微超调)
- BEST_EFFORT QoS: 丢失的消息静默丢弃, 无重传

---

## 总结: VR桥接器管道

### 数据流:
```
VR设备 (WebXR)                           VR原生坐标, 60 Hz
    |
    v HOP 0: HTTPS POST /poseData        原始JSON字符串
    |
    v HOP 1: json.loads()               Python字典
    |
    +--> HOP 2: _make_pose()            geometry_msgs/PoseStamped × 3
    |
    +--> HOP 3: _make_buttons()         std_msgs/Float32MultiArray × 2
    |
    v HOP 4: DDS发布                    /vr/* ROS2话题
```

### 关键参考文档:
1. **WebXR Device API规范**: `XRPose`——位置单位为米, 朝向为xyzw四元数
2. **teleoperation_interface/README.md**: 按钮通道映射, 话题命名约定
3. **vr_bridge_server.py:70-77**: QoS配置常量
4. **config/config.json**: 端口, IP, 数据速率

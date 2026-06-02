# 相机流管道映射 — 调试参考

日期: 2026-06-02

完整管道: RealSense相机 → 帧读取 → RGB→BGR → JPEG编码 → ROS2 Image + CameraInfo发布

---

## HOP 0: 相机发现与连接

**维度空间**: USB设备 → 已连接的RealSense对象

```
camera_publisher.py:115-140
```
```python
    for idx, info in enumerate(found[:max_cameras]):
        if idx >= len(config_keys):
            continue
        cam_key = config_keys[idx]
        entry = cameras_cfg[cam_key]
        sn = info.get("id", "")
        cfg_sn = entry.get("serial", "auto")
        if cfg_sn != "auto" and cfg_sn != sn:
            continue
        topic = entry.get("topic", f"/camera/{cam_key}")
        try:
            cam_cfg = RealSenseCameraConfig(serial_number_or_name=sn)
            cam = RealSenseCamera(cam_cfg)
            cam.connect()
            connected.append((cam_key, topic, entry, cam))
        except Exception as e:
            continue
```

**公式**: 无 (结构性的——设备枚举与连接)

**说明**: 通过lerobot SDK发现已连接的RealSense相机, 按发现顺序匹配配置条目, 建立连接。每个相机由其序列号标识。若配置中`serial: "auto"`, 则使用该发现索引处的任何设备。序列号不匹配的相机被跳过并警告。参考来源: `config/camera.yaml` `cameras:`节。

### 功能描述

枚举已连接的RealSense相机, 按发现顺序匹配`config/camera.yaml`中的条目, 连接每个相机。序列号不匹配的设备被跳过并警告。

### 输出示例

**输入**: `RealSenseCamera.find_cameras()` 返回3个设备
**输出**:
```
connected = [
    ("right_wrist", "/camera/right_wrist", <条目字典>, <RealSenseCamera对象>),   ← D405 #0
    ("head",       "/camera/head",       <条目字典>, <RealSenseCamera对象>),   ← D435 #1
    ("left_wrist", "/camera/left_wrist", <条目字典>, <RealSenseCamera对象>),   ← D405 #2
]
```

关键特征:
- 发现顺序决定配置匹配——第一个相机获得第一个配置条目
- `serial: "auto"`接受该索引处的任何设备
- 序列号不匹配产生`[WARN]`并跳过, 不崩溃

---

## HOP 1: 帧读取

**维度空间**: 相机硬件 → numpy数组 (RGB, uint8)

```
camera_publisher.py:248-250
```
```python
    frame = s["cam"].read()
    if frame is None or frame.size == 0:
        continue
```

**公式**: `F ∈ Z_uint8^{H×W×3}_RGB` 其中 `H=480, W=640` (可配置)

**说明**: 通过`cam.read()`从RealSense相机读取一帧。这是一个阻塞调用, 以相机原生速率返回numpy数组——不受定时器限速。底层lerobot SDK封装`pyrealsense2`, 随硬件帧到达而传送。参考来源: 相机硬件规格——D405单流30 Hz, D435复合流约15 Hz(4流)。

### 功能描述

从已连接的RealSense相机读取单帧RGB数据为numpy uint8数组。在后台线程中以原生相机速率运行——无人为的`create_timer`上限。

### 输出示例

**输入**: 在D405上以原生速率调用`cam.read()`
**输出**:
```
frame.shape = (480, 640, 3)   ← H × W × RGB
frame.dtype = uint8           ← 每通道0-255
frame.size  = 921600          ← 480 × 640 × 3 字节
```

关键特征:
- 单流: ~30 Hz。多流(深度+红外+彩色): ~15 Hz (USB 3.2带宽限制)
- 相机断开或丢帧时返回`None`或空数组
- `cam.read()`阻塞直至下一帧可用

---

## HOP 2: RGB → BGR转换

**维度空间**: RGB uint8 → BGR uint8

```
camera_publisher.py:252-255
```
```python
    if len(frame.shape) == 3 and frame.shape[2] == 3:
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    else:
        frame_bgr = frame
```

**公式**: `B[i,j,0]=R[i,j,2], B[i,j,1]=R[i,j,1], B[i,j,2]=R[i,j,0]` (通道交换)

**说明**: OpenCV的`imencode`期望BGR顺序。RealSense SDK输出RGB。此步骤交换通道0↔2。灰度或非3通道帧原样传递。参考来源: OpenCV约定——`cv2.imencode`要求BGR输入。

### 功能描述

使用`cv2.cvtColor`将RGB帧转换为BGR。必需, 因为OpenCV的JPEG编码器期望BGR通道顺序。非3通道帧原样传递。

### 输出示例

**输入**: `frame[0,0] = [255, 128, 64]`  (RGB: 红=255, 绿=128, 蓝=64)
**输出**: `frame_bgr[0,0] = [64, 128, 255]`  (BGR: 蓝=64, 绿=128, 红=255)
                                                  ^^^^^^^^^ 通道顺序反转

关键特征:
- 仅当`frame.ndim == 3`且`frame.shape[2] == 3`时应用
- 原地通道交换, 无额外内存分配
- 单色或深度帧(2D)原样传递

---

## HOP 3: JPEG编码

**维度空间**: BGR uint8 numpy → JPEG字节

```
camera_publisher.py:257-262
```
```python
    ret, jpeg_buf = cv2.imencode(".jpg", frame_bgr,
        [cv2.IMWRITE_JPEG_QUALITY, s["jpeg_q"]])
    if not ret:
        continue
    jpeg_bytes = jpeg_buf.tobytes()
```

**公式**: `B_jpeg = encode_JPEG(F_BGR, Q)` 其中 `Q ∈ [1, 100]` 为质量 (默认70)
`|B_jpeg| ≈ 20-30 KB` (640×480, Q=70时)

**说明**: 使用OpenCV编码器将BGR帧压缩为JPEG。质量在`camera.yaml`中配置(`jpeg_quality: 70`)。若编码失败(`ret == False`), 帧被静默丢弃。JPEG编码将640×480×3 = 921 KB原始数据减至约27 KB压缩数据——34:1的压缩比。JPEG字节原样存入ROS2 Image消息(写入端不重新编码)。参考来源: `config/camera.yaml` `jpeg_quality`键。

### 功能描述

以配置的质量等级将BGR帧压缩为JPEG。若编码失败, 帧被丢弃。JPEG编码帧由数据采集写入端原样存储。

### 输出示例

**输入**: `frame_bgr`: (480, 640, 3) uint8, `jpeg_quality = 70`
**输出**:
```
jpeg_bytes = bytes, 长度 ~27,000         ← 变长, 70%质量约27 KB
jpeg_bytes[0:2] = b'\xff\xd8'           ← JPEG SOI魔数 (HDF5验证器检查此项)
```

关键特征:
- 变长: 15-50 KB, 取决于场景复杂度
- 以`0xFFD8`(JPEG魔数)开头——采集后验证
- 编码失败返回`ret=False`→帧静默丢弃
- 数据采集写入端无重新编码——原样存储

---

## HOP 4: ROS2 Image消息构造

**维度空间**: JPEG字节 → `sensor_msgs/Image`

```
camera_publisher.py:264-273
```
```python
    msg = Image()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = "jpeg"
    msg.is_bigendian = 0
    msg.step = len(jpeg_bytes)
    msg.data = jpeg_bytes
    img_pub.publish(msg)
```

**公式**: `msg.data = B_jpeg`, `msg.encoding = "jpeg"`, `msg.step = |B_jpeg|`
`msg.header.stamp = t_encode` (编码时刻的ROS2时钟)

**说明**: 将JPEG字节封装为标准ROS2 `sensor_msgs/Image`消息。`encoding`字段为`"jpeg"`——告知所有下游消费者数据已预压缩。`step`字段设为字节长度(而非行步长), 因为JPEG无固定行大小。高度和宽度来自numpy帧形状。参考来源: `sensor_msgs/Image`消息定义——`encoding="jpeg"`表示预压缩数据。

### 功能描述

构造带JPEG编码数据的`sensor_msgs/Image`消息。设置encoding为`"jpeg"`使下游消费者(数据采集服务, rviz2)知晓数据已预压缩。高度/宽度反映原始帧尺寸。

### 输出示例

**输入**: `jpeg_bytes` (27 KB JPEG), `frame_id = "camera_right_wrist_optical_frame"`
**输出**:
```
header:
  stamp: {sec: 1780382756, nanosec: 456789012}        ← 编码时刻的ROS2时钟
  frame_id: camera_right_wrist_optical_frame          ← 来自config/camera.yaml
height: 480                   ← 原始帧高度
width: 640                    ← 原始帧宽度
encoding: "jpeg"              ← 表示预压缩数据
is_bigendian: 0
step: 27345                   ← 字节长度, 非行步长
data: [0xFF, 0xD8, ...]      ← 原始JPEG字节
```

关键特征:
- `step` = 字节长度(非行步长), 因为JPEG无行的概念
- `encoding: "jpeg"`触发数据采集写入端的"原样存储"行为
- `frame_id`来自`camera.yaml`中的每相机配置

---

## HOP 5: CameraInfo发布 (仅启动时)

**维度空间**: YAML内参 → `sensor_msgs/CameraInfo`

```
camera_publisher.py:212-215
```
```python
for topic, frame_id, _, _, info_pub in self._streams:
    cam_key = topic.rsplit("/", 1)[-1]
    ci = _build_camera_info(frame_id, self._intrinsics.get(cam_key, {}))
    info_pub.publish(ci)
```

**公式**: `K = [f_x, 0, c_x; 0, f_y, c_y; 0, 0, 1]`, `P = [K | 0]`, `R = I_3`
启动时发布一次, 使用`TRANSIENT_LOCAL`持久性(锁存)。

**说明**: 内部校准参数从`config/camera.yaml`读取, 在启动时作为`CameraInfo`发布一次。QoS为`TRANSIENT_LOCAL`——意味着后到的订阅者(包括在相机之后启动的数据采集服务)无需等待即可收到锁存消息。参考来源: `config/camera.yaml` `intrinsics:`节。

### 功能描述

在启动时将相机内部校准作为`CameraInfo`发布一次。使用TRANSIENT_LOCAL QoS使后到订阅者无需等待重发即可收到。在流循环开始前发布。

### 输出示例

**输入**: `config/camera.yaml`中的内参
**输出**:
```
K: [615.0, 0.0, 320.0, 0.0, 615.0, 240.0, 0.0, 0.0, 1.0]   ← fx, fy, cx, cy
P: [615.0, 0.0, 320.0, 0.0, 0.0, 615.0, 240.0, 0.0, ...]
distortion_model: "plumb_bob"
D: [0.0, 0.0, 0.0, 0.0, 0.0]   ← 零畸变 (默认值)
     ^^ k1   k2   t1   t2   k3
```

关键特征:
- 启动时仅发布一次, 非每帧
- TRANSIENT_LOCAL = 锁存——后到订阅者可接收
- 默认值(615, 320, 240)为占位符——替换为标定值

---

## 总结: 相机流管道

### 数据流:
```
RealSense相机                            原始RGB uint8, 640×480, 原生速率
    |
    v HOP 0: find_cameras + connect      已连接相机对象
    |
    v HOP 1: cam.read()                  numpy uint8数组 (H, W, 3)
    |
    v HOP 2: RGB → BGR通道交换            numpy uint8数组, BGR顺序
    |
    v HOP 3: cv2.imencode JPEG            JPEG字节 (~27 KB)
    |
    v HOP 4: Image消息构造                sensor_msgs/Image (jpeg)
    |
    +--> HOP 5: CameraInfo发布            sensor_msgs/CameraInfo (仅启动时, 锁存)
```

### 关键参考文档:
1. **config/camera.yaml**: 相机发现映射, JPEG质量, 内部校准
2. **sensor_interface/README.md**: Image + CameraInfo约定
3. **sensor_msgs/Image**: ROS2消息定义——encoding字段
4. **RealSense硬件规格**: D405 (30 Hz单流), D435 (~15 Hz复合)

# 数据采集服务管道映射 — 调试参考

日期: 2026-06-02

完整管道: ROS2话题 → 订阅回调 → 适配器解码 → 边界验证 → 录制门控 → 批缓冲区 → HDF5刷写 → 输出验证

完整的8-HOP追踪(含完整代码片段、公式和行号)请参见
`memodocs/pipeline_mapping_debug_reference.md`。本文档提供每个HOP的紧凑摘要和输出示例。

---

## HOP 0: 会话配置加载

**维度空间**: 磁盘上的YAML → 冻结的Python数据类

```
core/config/session_config.py:393-411
```
```python
def load_session_config(config_path: str | Path) -> SessionConfig:
    resolved = Path(config_path).expanduser().resolve()
    raw = yaml.load(resolved.read_text(encoding="utf-8"), Loader=_UniqueKeyLoader)
    return _parse_config(raw, source_path=str(resolved))
```

**公式**: 无 (结构性的——YAML解析, 带重复键拒绝)

**说明**: 从磁盘加载会话YAML, 拒绝重复映射键。解析为冻结的`SessionConfig`及`StreamConfig`条目。验证在加载时发生——加载后的`SessionConfig`保证有效。参考来源: `config/session/session_vr_ik_robot_button_control.yaml`。

### 功能描述

读取会话YAML文件, 验证每个字段, 产生不可变的`SessionConfig`对象。若任何字段缺失、拼写错误或超出范围, 在加载时失败。

### 输出示例

**输入**: `config/session/session_vr_ik_robot_button_control.yaml`
**输出**:
```
SessionConfig(
    schema_version="1.0",
    session=SessionMeta(name="vr_ik_robot_button_control", ...),
    storage=StorageConfig(root="data/episodes", format="hdf5", ...),
    streams={
        "vr_head_pose": StreamConfig(
            name="vr_head_pose", topic="/vr/head_pose",
            message_type="geometry_msgs/PoseStamped",        ← 标准ROS2类型
            time_domain=TimeDomain.ROS_HEADER,              ← 使用header.stamp
            qos=StreamQoS(reliability=best_effort, ...),   ← BEST_EFFORT
            fields=(FieldRule(path="pose.position.x", ...), ...)  ← 7条字段规则
        ),
        ...
    }
)
```

关键特征:
- 冻结数据类——加载后不可变
- 加载时约80条验证规则(枚举错误, 缺失字段, 负按钮索引)
- YAML重复键被拒绝(防止静默覆盖错误)

---

## HOP 1: 适配器注册表解析

**维度空间**: `SessionConfig.streams` → `dict[name, ConfiguredStreamAdapter]`

```
core/adapters/registry.py:123-124
```
```python
def resolve_session(self, config: SessionConfig) -> dict[str, ConfiguredStreamAdapter]:
    return {name: self.resolve(stream) for name, stream in config.streams}
```

**公式**: 无 (结构性的——`(source, message_type)`对查找)

**说明**: 为每个流查找匹配的`AdapterBinding`, 按`(source, message_type)`签名。每个绑定声明一个载荷配置和一个载荷构建函数。若有多个绑定匹配同一签名, 解析因歧义错误失败。参考来源: `registry.py:_default_bindings()`中的默认绑定。

### 功能描述

将每个流的`(source, message_type)`映射到知道如何将该ROS2消息类型解码为规范载荷的具体适配器。若流无匹配适配器, 在启动时失败。

### 输出示例

**输入**: 带有流`vr_head_pose` (source=teleop, type=PoseStamped)的`SessionConfig`
**输出**:
```
{"vr_head_pose": ConfiguredStreamAdapter(
    stream=StreamConfig(...),
    binding=AdapterBinding(
        source="teleop", message_type="geometry_msgs/PoseStamped",    ← 按(source, type)匹配
        payload_profile="teleop_pose",                                ← 按此配置文件验证
        payload_builder=extract_pose_payload                          ← 通用PoseStamped提取器
    ),
    validator=AdapterBoundaryValidator(...)
)}
```

---

## HOP 2: ROS2订阅创建

**维度空间**: YAML流配置 → 带QoS的活动ROS2订阅

```
core/runtime/ros2_collection_node.py:124-132
```
```python
    for name, stream in self._config.streams:
        msg_cls = _resolve_message_class(stream.message_type)
        if msg_cls is None:
            self.get_logger().warn(f"skipping {name}: cannot resolve {stream.message_type}")
            continue
        qos = _qos_profile_from_stream(stream)
        self.create_subscription(msg_cls, stream.topic, self._make_handler(name), qos)
```

**公式**: 无 (结构性的——DDS订阅注册, 带动态消息类型导入)

**说明**: 为每个流创建一个ROS2订阅。消息类从类型字符串动态导入(例如`"geometry_msgs/PoseStamped"` → `geometry_msgs.msg.PoseStamped`)。QoS配置从YAML配置翻译。不支持的类型被跳过并警告。参考来源: 会话YAML `streams:`节。

### 功能描述

为会话YAML中声明的每个流创建DDS订阅。消息类型动态解析——无硬编码类型列表。每个订阅路由到以流名称标识的闭包回调。

---

## HOP 3: 适配器边界 — 消息 → WriterSample

**维度空间**: 原始ROS2消息 → `WriterSample` (展平, 验证, 写入就绪)

```
core/runtime/ros2_collection_node.py:139-158
```
```python
    adapter = self._adapters.get(stream_name)
    if adapter is None:
        return
    try:
        now = datetime.now(timezone.utc)
        sample = adapter.adapt(msg, received_at=now)
        self._stream_tracker.record_valid(stream_name, sample.timestamp_ns)
```
```
core/adapters/registry.py:78-92
```
```python
    def adapt(self, message, *, received_at):
        payload = self.binding.payload_builder(message, self.stream)
        sample = build_boundary_sample(self.stream, payload=payload, ...)
        validated = self._validator.validate_sample(sample, ...)
        ts_ns = int(validated.timestamp.timestamp() * 1e9)
        return _writer_sample_from_payload(self.stream.name, validated.payload, ts_ns)
```

**公式**: 向量: `v = U_{leaf in payload} float(leaf)`, 深度优先, 按键排序。
图像: `image_data = payload["data"]` 若为bytes。
`ts_ns = floor(t_validated.timestamp() * 10^9)`

**说明**: 适配器边界——最关键的HOP。载荷构建器按YAML约定提取字段。边界验证器检查字段存在性、数值有限性(NaN/Inf)和数值范围。通用`_writer_sample_from_payload()`递归遍历已验证字典并产生展平的`WriterSample`。图像与向量的判断通过检查`"data"`是否为bytes——无按类型分派。时间戳来源由YAML中的`time_domain`选择: `ros_header`使用`header.stamp`, `ros_receive`使用到达时间。参考来源: 会话YAML中流的`FieldRule`定义。

### 功能描述

将ROS2消息解码为已验证、展平的`WriterSample`。按YAML约定提取字段, 按配置文件验证(NaN/Inf拒绝), 按`time_domain`选择时间戳, 展平为写入就绪格式。这是消息类型知识进入管道的唯一点。

### 输出示例

**输入**: `/vr/right_pose`上的`PoseStamped`消息
**输出**:
```
WriterSample(
    stream_name="vr_right_pose",
    timestamp_ns=1779954107356294491,             ← uint64纳秒纪元
    values=(0.201, 0.832, -0.356, 0.227, ...),  ← 7个浮点数: xyz + xyzw
    image_data=None                              ← 对向量为None
)
```

对于JPEG相机流:
```
WriterSample(
    stream_name="camera_right_wrist",
    timestamp_ns=1780382756456789012,
    values=None,                                  ← 对图像为None
    image_data=b'\xff\xd8\xff\xe0...'            ← 原始JPEG字节 (~27 KB)
)
```

关键特征:
- NaN或Inf值在边界被拒绝——永不进入HDF5
- `time_domain: ros_header`时缺少`header.stamp`导致`AdapterError`
- 展平完全通用——无按消息类型的代码路径

---

## HOP 4: 录制门控

**维度空间**: `WriterSample` → 被丢弃或传递

```
core/runtime/ros2_collection_node.py:133-134
```
```python
if not self._state_machine.is_recording:
    return
```

**公式**: `g(sample) = sample 若 state == RECORDING 否则 空集`

**说明**: 唯一的控制开关。所有三种控制模式(service, manual_ui, device_binding)切换同一个`is_recording`标志。未录制时, 已验证样本被静默丢弃——无缓冲, 无时间戳间隙。参考来源: `recording_state.py:RecordingStateMachine`。

### 功能描述

在未录制时静默丢弃所有已验证样本。管道中唯一的门控点——无同步窗口, 无必需流检查, 无过时样本拒绝。

---

## HOP 5: 写入器分派

**维度空间**: `WriterSample` → `writer.append_image()` 或 `writer.append_vector()`

```
core/runtime/ros2_collection_node.py:141-145
```
```python
if sample.image_data is not None:
    self._writer.append_image(stream_name, sample.image_data, sample.timestamp_ns)
elif sample.values is not None:
    self._writer.append_vector(stream_name, sample.values, sample.timestamp_ns)
```

**公式**: 无 (结构性的——节点外壳是透传; 所有展平在HOP 3完成)

**说明**: 节点外壳接收`WriterSample`并路由至写入器。它不知道消息类型、载荷形状或字段名称。它检查哪个字段被填充并调用相应的写入器方法。这是整个摄取逻辑——4行代码。参考来源: `contracts/writer_sample.py`中的`WriterSample`约定。

### 功能描述

根据填充的字段将`WriterSample`路由至写入器的append方法。节点外壳是4行透传——所有智能都在适配器中。

---

## HOP 6: 分块批缓冲

**维度空间**: 单个样本 → 缓冲的`list[ndarray]` + `list[uint64]`

```
core/storage/airs_hdf5_writer.py:153-160
```
```python
class _VectorBuffer:
    def append(self, values, timestamp_ns):
        arr = np.asarray(values, dtype=np.float32)
        if arr.ndim == 0: arr = arr.reshape(1)
        if arr.shape[-1] != self.dims and arr.size == self.dims:
            arr = arr.reshape(1, self.dims)
        self._data.append(arr.ravel().astype(np.float32))
        self._timestamps.append(np.uint64(timestamp_ns))
```

**公式**: 向量: `D_i = D_i-1 U {v_i}, T_i = T_i-1 U {t_i}`
图像: `F_i = F_i-1 U {b_i}, T_i = T_i-1 U {t_i}`

**说明**: 每个流有一个私有的内存批缓冲区。向量归一化为`float32`, 第一帧自动检测维度。图像可选地重新编码为JPEG。当批次达到阈值(向量50帧/图像20帧), `_flush_batch()`通过增量`resize()` + 连续切片写入将批次写入HDF5。内存使用恒定——任何时候仅一个批次在内存中。参考来源: AIRS标准——`data`数据集形状`(N, dims)`或`(N,)`, `timestamps`数据集形状`(N,)`。

### 功能描述

以小批次缓冲样本(向量50/图像20帧)。达到阈值时通过增量resize+write刷写到HDF5。无论录制时长, 内存使用恒定。

### 输出示例

**输入**: 50个连续的7维PoseStamped流的`WriterSample`值
**输出 (刷写后的HDF5中)**:
```
/stream/data:        (50, 7) float32       ← 批次7维向量
/stream/timestamps:  (50,)  uint64         ← 匹配的时间戳
```

关键特征:
- 缓冲区上限为50(向量)/20(图像)——恒定内存
- `dims=0`时第一帧自动检测维度
- 向量在缓冲区入口处强制转换为`float32`
- 刷写是增量的——`resize()` + 写入, 从不重建数据集

---

## HOP 7: 输出验证

**维度空间**: 磁盘上的HDF5文件 → `ValidationReport {is_valid, errors[], warnings[]}`

```
core/validation/dataset_validator.py:48-69
```
```python
def validate_dataset(dataset_path: str | Path) -> ValidationReport:
    with h5py.File(path, "r") as f:
        _validate_root_attrs(f, report)
        for group_name in f:
            gtype = str(f[group_name].attrs["type"])
            if gtype == "vector": _validate_vector_group(f, group_name, report)
            elif gtype == "image": _validate_image_group(f, group_name, report)
    return report
```

**公式**: 无 (结构验证——检查形状, 数据类型, 属性存在性, JPEG魔数)

**说明**: 对AIRS标准HDF5文件进行采集后验证。根属性必须存在。每个组必须具有`type`属性, 值为`vector`或`image`。验证数据类型(向量为`float32`, 时间戳为`uint64`), 形状一致性(data[0]必须等于timestamps[0])和JPEG魔数(第一帧开头为`0xFFD8`)。参考来源: AIRS标准`buildin.hdf5`格式。

### 功能描述

打开已完成的HDF5文件并验证每个组符合AIRS标准。检查根属性, 每组结构, 数据类型, 维度一致性和JPEG魔数。返回带错误和警告的结构化报告。

### 输出示例

**输出**:
```
Dataset: data/episodes/ep_20260525_143022.h5
Status: valid
Errors: 0
Warnings: 0
        ↑ 若>0: 文件格式错误, 下游训练可能失败
```

---

## 总结: 数据采集服务管道

### 数据流:
```
ROS2话题 (/vr/right_pose, /arm/left/joint_state, ...)     原始ROS2消息对象
    |
    v HOP 0: YAML → SessionConfig                           冻结数据类, 加载时验证
    |
    v HOP 1: AdapterRegistry.resolve_session()              dict[name, ConfiguredStreamAdapter]
    |
    v HOP 2: 每流create_subscription()                      带QoS的活动DDS订阅
    |
    v HOP 3: adapter.adapt(msg)                             WriterSample (展平, 已验证, ts_ns)
    |
    v HOP 4: is_recording?                                  门控: 非RECORDING则丢弃
    |
    v HOP 5: writer.append_image() 或 append_vector()       透传分派
    |
    v HOP 6: 分块批缓冲 → 达到阈值时刷写                      HDF5增量resize + 写入
    |
    +--> /<stream>/data                                      AIRS向量 (N, D) float32
    |
    +--> /<stream>/timestamps                                AIRS时间戳 (N,) uint64
    |
    +--> /<stream>/attrs                                     AIRS组元数据
    |
    v HOP 7: validate_dataset()                             ValidationReport {errors, warnings}
```

### 关键参考文档:
1. **AIRS标准**: HDF5布局——根属性, 每流组, `data[N]` + `timestamps[N]`, `type`属性
2. **会话YAML**: 每流约定——字段路径, 类型, 必需标志, QoS
3. **config/session_config.py**: 加载时约80条验证规则
4. **adapter_profiles.py**: 每种消息类型的载荷字段规则
5. **airs_hdf5_writer.py**: 分块追加写入器——50向量/20图像批次阈值

# 管道映射覆盖盲区

日期：2026-06-02

四份 PIPELINE_MAPPING.md 文件未覆盖的内容 — 每个数据 HOP 周边的代码。

---

## teleoperation_interface — `vr_bridge_server.py`

### 任何 HOP 均未提及：

**模块初始化（第30–63行）：** 通过 `LEROBOT_SRC` 环境变量解析 `ROOT` 路径，`resolve_path()` 辅助函数将相对路径映射到包根目录。若干导入守卫（`ROS2_AVAILABLE`、`HAS_AIOHTTP`）控制整个模块 — 如果依赖缺失，节点静默降级而非崩溃。

**日志配置（第65–78行）：** 一个轮转文件处理器写入 `logs/vr_bridge.log`，5 MB 滚动，保留 3 个备份。自定义 `ThrottledHTTPAccessLogger` 抑制默认的 aiohttp 访问日志噪音。

**`VRBridgeNode` 类的 `__init__`（第186–221行）：** 基于条件导入路径创建 6 个 ROS2 发布者 — 如果 `rclpy` 缺失，节点不会实例化，HTTP 服务器仍运行但静默丢弃姿态数据。发布者包括：`/vr_raw_data`（String 透传）、`/vr/head_pose`、`/vr/left_pose`、`/vr/right_pose`（PoseStamped）、`/vr/left_buttons`、`/vr/right_buttons`（Float32MultiArray）。所有发布者共享 `REALTIME_QOS`。

**速率统计（第247–253行）：** 每 5 秒，节点记录观测到的发布速率。这是一个轻量级带内遥测检查 — 如果与预期的约 60 Hz 偏差，则 VR 设备或网络可能是问题根源。

**`handle_index`（第304–309行）：** 在 `/` 路径上提供静态 VR HTML 页面。使用 `resolve_path()` 查找 `static/VRDriverPlaneCamera.html`。如果缺失则返回 404。

**`main()`（第312–407行）：** 入口点。加载 `config/config.json` 获取端口/IP，有条件地初始化 ROS2（在守护线程中运行 `rclpy` spin），启动 aiohttp web 应用，将 `handle_pose` 注册到 `POST /poseData`，`handle_index` 注册到 `GET /`。设置 SIGINT/SIGTERM 信号处理器。在关闭时清理 ROS2 和事件循环。

**信号处理：** 注册 `signal.signal(signal.SIGINT, ...)` 以优雅地停止 asyncio 循环和 ROS2 spin 线程。

---

## sensor_interface — `camera_publisher.py`

### 任何 HOP 均未提及：

**导入守卫（第30–50行）：** `_HAS_REALSENSE`、`_HAS_CV2`、`_HAS_ROS2` — 每个守卫保护对应的缺失依赖。如果 `cv2` 缺失，流循环变为 `time.sleep(0.1)` 空操作而非崩溃。

**`_DIRECT_API_STREAMS`（第55行）：** 一组需要直接 `pyrealsense2` API 访问的流类型（`{"depth", "infra_left", "infra_right"}`），明确不支持。在发现过程中会警告但从不启用。

**QoS 配置文件（第57–68行）：** 两个独立的 QoS 配置：`_QOS` 用于 Image 主题（BEST_EFFORT，KEEP_LAST depth=1），`_CAMERA_INFO_QOS` 用于 CameraInfo（TRANSIENT_LOCAL — 锁存模式，使延迟订阅者能收到最后发布的内参）。

**`_build_camera_info()`（第157–171行）：** 构造完整的 `CameraInfo` 消息，包含 K（内参矩阵）、P（投影矩阵）、R（校正矩阵）和 D（畸变系数）矩阵。默认值：fx/fy=615，cx=320，cy=240，plumb_bob 模型且零畸变。这些默认值是占位符 — 实际标定值来自 `config/camera.yaml`。

**`CameraPublisherNode.__init__` 完整内容（第181–228行）：** 远不止发现调用。遍历每个摄像头的流配置，为 `Image` 和 `CameraInfo` 主题分别创建发布者，按后缀约定命名主题（`image_raw`、`depth/image_raw` 等），并将每个流的配置存储在字典中（`jpeg_q`、`encoding`、`stype`）。同时在启动时发布 CameraInfo。

**`start()` / `stop()`（第230–238行）：** 线程生命周期管理。`start()` 生成运行 `_stream_loop` 的守护线程。`stop()` 设置标志并以 2 秒超时 join。

**`_stream_loop()` 完整内容（第240–284行）：** 外层循环每次迭代遍历 `self._streams`（没有每个流独立的线程）。这很重要 — 所有摄像头共享一个线程，因此一个慢速摄像头会阻塞其他摄像头。包含 `_HAS_CV2` 守卫、每个流的异常处理且错误日志有 5 秒节流，以及调用 `_maybe_log_stats()` 进行周期性速率报告。

**`_maybe_log_stats()`：** 每 5 秒记录每个摄像头的帧数用于监控。

**`main()`：** 参数解析（`--config`），从 YAML 加载配置，发现 + 连接，节点创建，spin，以及清理。解析配置文件的 `cameras:` 和 `intrinsics:` 部分。

**串号匹配逻辑（第122–127行）：** 不仅仅是"连接任意摄像头" — 如果配置中的 `serial` 不是 `"auto"`，则发现的摄像头串号必须匹配，否则跳过并发出警告。这防止了连接到错误的物理设备。

---

## robot_interface

### `vr_subscriber.py` — 未提及：

**`VRPose` 和 `VRButtonState` 数据类：** 不可变数据容器，用于姿态（位置 xyz + 方向 xyzw + 时间戳）和按钮状态（6 个浮点数 + 时间戳）。

**`VRDataStore` 类（完整内容，第29–136行）：** 位于 ROS2 回调和求解器循环之间的线程安全存储。已映射方法之外的其余方法：`update_pose()`、`update_buttons()`、`update_raw()`、`get_latest_poses()`（具有过期超时过滤，仅返回未过期的姿态）、`get_head_pose()`、`get_left_buttons()`、`get_right_buttons()`、`is_connected()`（如果在 `stale_timeout_s` 秒内无消息则返回 False）、`get_status()`（用于 WebSocket 广播的完整快照）。过期超时至关重要 — 它防止 IK 求解器在 VR 设备断开连接时基于航位推测的姿态继续运行。

**`VRSubscriber._run_ros2()`（第184–230行）：** 实际的守护线程函数。创建 `rclpy` 节点，以 BEST_EFFORT QoS 设置 5 个订阅，并调用 `rclpy.spin()`。每个订阅回调解析 ROS2 消息并调用对应的 `data_store.update_*()` 方法。在 `KeyboardInterrupt` 或 `rclpy` 错误时优雅关闭。这是 HOP 0 总结的真正订阅逻辑。

**订阅回调：** 解析 `PoseStamped` → `VRPose`，`Float32MultiArray` → 按钮列表，`String` → 原始 JSON 字典。每个回调剥离 ROS2 包装层，仅将有效载荷存入 `VRDataStore`。

**`is_connected` / `ros2_installed` / `is_running` 属性：** 求解器循环用于判断是否回退到空闲/键盘控制的状态标志。

---

### `solver_loop.py` — 未提及：

**外层 `run_solver_loop()` 函数（第24–170行，仅 HOP 1–4 被映射）：** 一个 `aiohttp` 应用生命周期函数。以 IK 服务的 `on_startup`/`on_cleanup` 钩子开始。包含两个映射未覆盖的额外控制路径：

**优先级 2：传统目标缓冲区（第79–88行）：** 当 VR 未激活时，求解器可以接受来自基于拖拽的 WebSocket UI 的目标。`buffer.read()` 拉取由不同接口设置的左/右目标，通过 `parse_target()` 解析。这是"拖拽"控制源。

**控制源状态机：** `"idle"` → `"vr"`（优先级 1） → `"drag"`（优先级 2）。当两个源都静默时，不调用求解器（`result = None`），循环仍保持 50 Hz 节奏。

**VR 状态构建（第97–115行）：** 为 WebSocket 快照广播构建丰富的状态字典。调用 `_map_vr_poses()` 将 VR 原生坐标变换到机器人基座坐标系（与标准化器分离 — 这是为了可视化，而非 IK）。包含标定状态、订阅者健康和 ROS2 可用性。

**`_broadcast()`（第172–182行）：** 快照广播路径 — 将完整系统状态（求解结果、VR 状态、求解器健康、控制源、标定状态）作为 JSON 发送到所有通用 WebSocket 客户端。这与仅发送关节命令的 `_broadcast_arm()` 是分离的。

**`build_snapshot()` 调用（第131行）：** 从求解结果、配置、VR 状态和求解器元数据组装完整的状态快照。这是 Web UI 的主要可视化数据。

**`_map_vr_poses()` 和 `_map_calibration()`：** 将 VR 姿态从原生空间变换到机器人基座坐标系用于可视化目的（不用于 IK — 标准化器处理那部分）。

**B 按钮复位到原位（第70–71行）：** 当 VR 标准化器从 `waiting` 转换到 `ready`（操作员按下 B 按钮固定原点），IK 求解器复位到原位。这是一个关键的 UX 行为 — 固定 VR 原点同时将机器人手臂重新居中。

**50 Hz 节奏睡眠（第165–169行）：** `SOLVE_PERIOD_S = 0.02` — 循环计算已用时间并睡眠剩余时间。如果求解耗时超过 20 ms，跳过睡眠（无负睡眠）。这保持了无论求解时间如何变化的一致控制速率。

**`_wxyz_to_xyzw()` 辅助函数：** 将四元数约定从求解器内部的 WXYZ 转换为 ROS2 的 XYZW。如果映射错误，坐标约定 bug 就藏在这里。

---

### `ros2_publisher.py` — 未提及：

**`ROS2Publisher` 类的 `__init__`（第108–143行）：** 在 `SingleThreadedExecutor` 中创建隐藏的 ROS2 节点，以 BEST_EFFORT QoS 设置 4 个发布者（`/arm/left/joint_commands`、`/arm/right/joint_commands`、`/arm/left/target_pose`、`/arm/right/target_pose`），并从配置中存储关节名称列表。

**线程化 spin 循环（第150–175行）：** 在守护线程中运行。使用 `_has_data` 标志和 `_latest` 元组将求解器循环的 `publish()` 调用与 DDS 写入解耦。`spin_once(timeout_sec=0.02)` 非阻塞地处理待处理的 DDS 工作。`_publish_pose()` 方法为目标姿态构造 `PoseStamped` — 这与 `_publish_joint_state` 模式相同，但用于姿态目标。

**`publish()` 方法（第72–93行）：** 求解器循环调用的接口。在锁的保护下存储最新的关节/姿态数据并设置 `_has_data = True`。spin 线程在下次迭代时拾取。

**线程启动/停止（第95–106行）：** `start()` 生成守护线程。`stop()` 设置 `_thread = None`，spin 循环在下次迭代时退出。

---

### `arm_state_publisher.py` — 未提及：

**`ArmStatePublisherNode.__init__`（第64–80行）：** 为 `/arm/left/joint_state` 和 `/arm/right/joint_state` 创建发布者，以配置的 `arm_state_hz`（默认 20 Hz）设置 `create_timer`，并从配置加载关节名称列表。关节名称默认为 `left_joint_1` 到 `left_joint_8`（8 个关节：7 个手臂 + 夹爪）。

**`_callback()` 完整内容（第82–106行）：** 在已映射的代码片段之外：用 try/except 包装 `_read_arm_state()` 并进行节流错误日志（5 秒节流），发布左右 `JointState` 消息，递增帧计数器，并调用 `_maybe_log()` 进行周期性速率报告。

**`_maybe_log()`：** 每 5 秒记录实际发布速率。

**`stop_timers()`（第108–112行）：** 在关闭时销毁所有定时器，防止节点销毁后回调继续触发。

**QoS 配置文件（第37–47行）：** BEST_EFFORT，KEEP_LAST depth=1 — 与命令发布者相同的模式。丢弃的状态读数是可以接受的。

**`main()`（第115–138行）：** 通过 `OpenArmsFollower` 连接到 CAN 总线（无标定，无力矩 — 纯观察者），创建节点，spin，并清理。follower 连接是关键步骤 — 如果 CAN 接口错误，所有状态读数都是零。

**配置驱动的关节命名（第71–72行）：** 关节名称来自配置而非硬编码。这意味着 8 关节约定（7 个手臂 + 夹爪在一条消息中）可针对每个部署进行配置。

---

## data_collection_service

### `session_config.py` — 未提及：

**`StreamConfig`、`FieldRule`、`TimeDomain`、`StreamQoS`、`SessionMeta`、`StorageConfig`、`RecordingControlConfig`：** 构成会话模式的所有冻结数据类。`FieldRule` 支持 `path`、`type`（标量/序列）、`min`/`max` 边界和 `required` 标志。

**`_parse_config()`（完整的解析器，约 200 行）：** 根据允许的键集合（`_ROOT_KEYS`、`_STREAM_KEYS`、`_SESSION_KEYS`）验证每个 YAML 键。拒绝未知键。通过 `TimeDomain.from_string()` 解析枚举。验证 `QoSProfile` 字段（可靠性、持久性、历史、深度）。每个 `FieldRule` 验证路径语法、允许的类型和范围一致性。未知的录制控制模式被拒绝。这就是文档中提到的约 80 个验证检查。

**`_UniqueKeyLoader`：** 一个拒绝重复映射键的 YAML 加载器子类。这防止了一类静默 bug — YAML 中重复的键会静默覆盖第一个值。

**`load_session_config_dict()`：** 备选入口点，接受预解析的字典而非文件路径。用于程序化配置生成/测试。

---

### `registry.py` — 未提及：

**`AdapterBinding` 和 `ConfiguredStreamAdapter`：** 绑定将 `(source, message_type)` 签名链接到 `payload_profile` 和 `payload_builder` 可调用对象。配置后的适配器将绑定与流配置和验证器包装在一起。

**`register_streams()`（第50–76行）：** 在 episode 创建期间调用以预注册 HDF5 组。通过检查 `message_type` 是否存在图像模式来判断流是图像还是向量。对于向量，从字段计数自动检测维度，除非存在序列字段（→ `dims=0` 用于运行时调整大小）。对于图像，注册宽/高/通道并可选地启用 JPEG 重新编码。

**`_resolve_binding()`（第127–140行）：** 通过 `(source, message_type)` 签名查找。如果多个绑定匹配，引发 `AdapterError` 并列出歧义列表。如果没有匹配，引发异常并列出支持的签名列表。这是启动时验证 — 配置错误的会话 YAML 立即失败。

**`_default_bindings()`（第157–208行）：** 所有支持的消息类型的注册表。每个条目将 `(source, type)` 对映射到有效载荷构建器函数。当前定义：`teleop_pose`（PoseStamped → 7 个字段：x,y,z + xyzw）、`robot_joint_state`（JointState → 8 个字段：7 个关节 + 夹爪）、`teleop_buttons`（Float32MultiArray → 6 个字段）、`camera_image`（Image → bytes）。这是添加新消息类型的地方。

**`AdapterBoundaryValidator` 和 `AdapterPayloadProfileRegistry`：** 验证器根据命名配置文件检查提取的有效载荷 — 每个配置文件定义了期望的字段、类型和可选的范围约束。

**`extract_header_timestamp()`：** 从有 header 的 ROS2 消息中提取 `header.stamp`。对于没有 header 的消息（如 `Float32MultiArray`）返回 `None`。

**`build_boundary_sample()` 和 `_writer_sample_from_payload()`：** 展平函数。递归遍历已验证的字典，对于向量按排序键提取浮点叶子节点，对于图像提取原始字节。

---

### `ros2_collection_node.py` — 未提及：

**`RecordingCollectionNode.__init__` 完整内容（第60–121行）：** 远不止创建订阅。创建：
- `RecordingStateMachine`，带有调用 `writer.open_episode()` 和 `writer.close_episode()` 的开始/结束处理器
- `StreamTracker`，具有可配置的期望周期（默认 500 ms） — 追踪每个流的有效/无效/延迟计数
- `RecordingControlRouter`，分发到三种控制模式之一（service、manual_ui、device_binding）
- `AdapterRegistry.with_defaults()` 并解析所有流适配器
- 手动操作员 UI（基于 aiohttp 的 web 服务器，监听可配置的 host:port）

**`RecordingStateMachine` 交互：** 开始处理器创建带有会话名称和时间戳的新 episode 目录。结束处理器关闭 HDF5 文件并记录最终路径和帧数。

**`StreamTracker`：** 追踪每个流的 `valid_count`、`invalid_count`、`late_count`。`record_valid()` 检查到达间隔是否超过期望周期并递增延迟计数器。这些数据在操作员 UI 和状态服务中暴露。

**`RecordingControlRouter`：** 三种模式：
- `service`：ROS2 服务 `/data_collection/record_start`、`/data_collection/record_stop`、`/data_collection/record_status`
- `manual_ui`：基于 web 的操作员仪表板，带有开始/停止按钮和每个流的健康状态
- `device_binding`：绑定到特定流消息（例如，VR 按钮按下切换录制状态）

**`_create_services()`：** 注册 ROS2 服务端点。

**`_start_manual_ui()`：** 在配置的端口上启动基于 aiohttp 的操作员仪表板。提供每个流的状态、录制状态和控制按钮。

**`_resolve_message_class()`：** 从字符串名称动态导入 ROS2 消息类型（例如，`"geometry_msgs/PoseStamped"` → `geometry_msgs.msg.PoseStamped`）。这是可扩展性机制 — 添加新消息类型不需要代码更改，只需一个新的适配器绑定。

**`_qos_profile_from_stream()`：** 将 YAML QoS 配置（字符串枚举：可靠性、持久性、历史）翻译为 `rclpy.qos.QoSProfile` 对象。

**Episode 生命周期：** `writer.open_episode()` 创建 HDF5 文件，注册所有组，并写入根属性。`writer.close_episode()` 最终化文件。两者都由 `RecordingStateMachine` 回调触发。

---

### `airs_hdf5_writer.py` — 未提及：

**`AirsHdf5Writer` 类（第30–157行）：** 完整的写入器生命周期：
- `open_episode()`：创建 HDF5 文件，写入根属性（`description`、`robot_type`、`series_number`、`sample_rate`、`frames`），并为所有注册的流组预创建其 `type` 属性。
- `close_episode()`：刷新所有缓冲区，将最终帧数写入根属性，关闭文件。
- `register_vector_stream()` / `register_image_stream()`：创建带有维度、列和编码参数的 `_VectorBuffer` / `_ImageBuffer` 实例。
- `append_vector()` / `append_image()`：类型检查并路由到正确的缓冲区。

**`_ImageBuffer` 类：** 与 `_VectorBuffer` 并行。存储原始字节（而非 numpy 数组）用于 JPEG 帧。可选地在追加时重新编码图像为 JPEG。有自己的批处理阈值（`_IMAGE_BATCH = 20`）。

**`_flush_batch()` 完整内容：** 对于向量：通过 `np.array()` 将缓冲的数组合并成 `(N, dims)`，在第一次刷新时惰性创建 `data` 和 `timestamps` 数据集（`maxshape=(None, dims)` 用于 data，`maxshape=(None,)` 用于 timestamps），然后增量 `resize()` + 写入新切片。图像同理。这种增量 resize 方法意味着 HDF5 文件始终有效 — 采集中途崩溃留下可读的部分文件。

**`_VECTOR_BATCH = 50` 和 `_IMAGE_BATCH = 20`：** 刷新阈值。在内存使用和 HDF5 写入放大之间取得平衡。

**`AirsHdf5WriterError`：** 维度不匹配和类型错误的自定义异常。

---

### `dataset_validator.py` — 未提及：

**`_validate_root_attrs()`（第71–75行）：** 检查必需的根属性：`description`、`robot_type`、`series_number`、`sample_rate`、`frames`。缺失属性报告为错误。

**`_validate_vector_group()`（第78–100行）：** 检查：`data` 数据集存在且 dtype 为 `float32`，`timestamps` 数据集存在且 dtype 为 `uint64`，两者第一维长度匹配，`columns` 属性与数据第二维匹配，数据中无 NaN 或 Inf 值。

**`_validate_image_group()`（第102–128行）：** 检查：`data` 数据集存在（dtype 灵活），`timestamps` 数据集存在且 dtype 为 `uint64`，长度匹配，第一帧以 JPEG 魔数字节 `0xFFD8` 开头。

**`ValidationReport` 和 `ValidationIssue`：** 结构化输出类型。`ValidationReport` 包含 `dataset_path`、`is_valid`（布尔值）、`errors`（列表）、`warnings`（列表）。`ValidationIssue` 包含 `path`（HDF5 组路径）和 `message`。

**`_ROOT_ATTRS`：** 必需的根属性规范集合。在此处添加新属性会使其对所有未来的 episode 强制执行。

---

## 覆盖盲区汇总

| 模块 | 主要未映射区域 |
|--------|---------------------|
| `vr_bridge_server.py` | 模块初始化、日志配置、`main()`、信号处理、路由注册、速率统计 |
| `camera_publisher.py` | 线程生命周期、`_build_camera_info()`、串号匹配逻辑、`main()`、每个流的配置、QoS 配置文件、多流类型支持 |
| `vr_subscriber.py` | VRDataStore（完整）、`_run_ros2()` 守护线程、订阅回调、过期超时逻辑、连接健康 |
| `solver_loop.py` | 外层循环、拖拽控制路径、VR 状态构建、快照广播、B 按钮复位到原位、四元数约定转换、控制源状态机 |
| `ros2_publisher.py` | 线程化 spin 循环、`_publish_pose()`、基于标志的数据解耦、线程生命周期 |
| `arm_state_publisher.py` | 基于定时器的循环、CAN follower 连接、`main()`、配置驱动的关节命名 |
| `session_config.py` | 完整的 `_parse_config()`（约 200 行，约 80 个验证）、所有数据类定义、`_UniqueKeyLoader` |
| `registry.py` | `_default_bindings()`、`_resolve_binding()`、`register_streams()`、有效载荷展平函数、边界验证器 |
| `ros2_collection_node.py` | Episode 生命周期、RecordingStateMachine、StreamTracker、RecordingControlRouter（3 种模式）、操作员 UI、ROS2 服务 |
| `airs_hdf5_writer.py` | 完整写入器生命周期、`_ImageBuffer`、`_flush_batch()`（增量 resize）、episode 打开/关闭、根属性写入 |
| `dataset_validator.py` | 每个组的验证函数（向量 dtype/shape、图像魔数字节）、`ValidationReport` 结构 |

管道映射覆盖了数据变换路径（约占总代码的 30%）。其余约 70% 是：生命周期管理（初始化/启动/停止/关闭）、健康监控（速率统计、连接超时、流追踪）、多模式控制（VR vs 拖拽 vs 空闲，service vs UI vs device_binding）、错误处理（导入守卫、异常节流、部分文件安全）以及配置/验证机制。

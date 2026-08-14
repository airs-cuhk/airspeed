# Airspeed 多 OS 兼容重构：Gap 探索清单（2026-08-14）

> 依据：`hardware-computational-requirements-20260814.md`（以下简称"需求文档"，file:line 证据见其内）。
> 目标：评估把 Airspeed 从"仅 Linux x86_64"重构为兼容 **Linux（x86_64 + aarch64）、Windows，远期 HarmonyOS** 所需提前探索和确认的 gap。
> 用法：每个 gap = 现状 → 差异 → **需要先确认的问题** → 验证方式（spike）。先跑 spike 再排期，不要凭直觉估工作量。

## 0. 先定策略：三种"兼容"不是一个价

| 策略 | 含义 | 适用 |
|---|---|---|
| A. 原生移植 | 全栈直接跑在目标 OS | Linux aarch64 可行；Windows 部分可行；HarmonyOS 不可能 |
| B. 服务化拆分 | 计算节点仍是 Linux，其余 OS 只做客户端/远程服务 | HarmonyOS 唯一现实路径；也降低 Windows 工作量 |
| C. 传输层替换 | 把 ROS2/DDS 换成跨平台传输（zenoh/WS） | 长期架构解，工作量最大 |

**建议在动工前全队先确认走哪条**：需求文档已给出大量服务化基础（5200 WS IK server、8765 SSE dashboard、ROS2→JSON→WS 中继、zenoh side channel），B 策略的骨架已经存在。

---

## 1. CAN 总线层（最大硬 gap）

**现状**：双臂（CAN FD 1M/5M）与双手（classic CAN 1M）全部硬编码 `can.interface.Bus(interface="socketcan")`（`damiao.py:160-171`、`socet_can_interface.py:106`）；`setup_can.sh` 用 `ip link`/`udevadm`。

**差异**：
- Windows：无 SocketCAN；PEAK 有 PCAN-Basic API（vendored OHand SDK 文档提及），python-can 有 `pcan` 后端
- HarmonyOS：无 CAN 栈、无 PEAK 驱动——**原生不可能**，只能 USB-CAN 网关转串口/网络，或 CAN 透传盒
- macOS：已有的 `slcan` 分支（`damiao.py:178-183`）实测是死代码（motor-register-audit 文档确认）

**需提前确认**：
1. python-can `pcan` 后端在 Windows 上对 **CAN FD** 的支持成熟度——整条手臂总线均为 CAN FD（1M/5M，涵盖全部 Damiao 电机型号，非仅 DM8009），FD 支持必须实测；
2. CAN 抽象层切口有多大：grep 全部 `socketcan`/`can.interface` 触点，确认除两个文件外是否还有隐藏依赖（如 `setup_can.sh` 的 udev 绑定逻辑）；
3. 单进程独占 CAN 通道的约束（`HAND_RESP_UNMATCHED_CMD`）在 Windows 驱动下是否同样成立；
4. HarmonyOS 侧是否存在任何可用的 USB-CAN 透传方案（CH343 类串口-CAN 桥在 OpenHarmony 有驱动先例——我们黑客松板载电机就是 CH343 串口）。

**验证 spike**：Windows + PCAN-Basic + python-can，单手臂 50Hz MIT 批指令 30 分钟延迟/丢帧实测。

## 2. ROS2 运行时与 Python 绑定

**现状**：ROS2 Humble @ `/opt/ros/humble`，系统 Python 3.10 编译绑定，bash launch 脚本全部 `source /opt/ros/...`（需求文档 §1）。

**差异**：
- Windows：ROS2 Humble 有 Windows 版但官方支持弱（Jazzy 起已基本放弃 Windows）；Python 绑定版本与系统 Python 布局不同；无 `/opt/ros` 路径
- HarmonyOS：完全没有 ROS2 运行时
- Linux aarch64：Humble 有 arm64 deb（Ubuntu 22.04 ARM），这层没问题

**需提前确认**：
1. Windows 上选哪条 ROS2 路线：官方二进制 vs **RoboStack（conda）**——社区主流答案是后者，需 spike 验证 rclpy + sensor_msgs + FastDDS 在 conda 下行为一致；
2. 仓库里硬编码 `/opt/ros/humble`、`python3.10/site-packages` 的所有位置（`server/main.py:33` 只是已知的一处），做一次全量 grep 盘点；
3. Windows 上 FastDDS **共享内存传输的 per-user 约束**是否仍然成立——注意该约束源自 Linux/Unix domain socket 语义（需求文档 §6：所有节点必须同用户且非 root）；Windows 上 FastDDS 通常退化为 UDP loopback，此约束大概率不适用，但需在 spike 中确认实际传输行为一致；
4. ROS2 Humble **Windows 版的 Python 绑定版本**与仓库假设的"系统 Python 3.10 布局"不一致——若 Spike 2 走通 RoboStack 则此项基本消解，但需先记录在案；
5. HarmonyOS 侧替代：rclpy 完全缺席时，是否用 **zenoh**（仓库已有 side channel 先例）或现有 ROS2→JSON→WS 中继做协议桥——这决定 HarmonyOS 是"客户端"还是"永远不可能"。

**验证 spike**：RoboStack 环境起最小节点对（talker/listener + 一个真实 msg 类型），确认话题发现与 QoS 行为与 Linux 一致。

## 3. IK 计算栈（x86_64 二进制依赖）

**现状**：IK adaptor 是 ~700MB x86_64 二进制包（`.pydeps` + JAX JIT 缓存），jax[cpu] 强制，50Hz 双臂带自碰（需求文档 §3）。

**差异**：
- aarch64：jaxlib 有 aarch64 wheel；pyroki/jaxlie 纯 Python 理论可跑；**pinocchio 需要源码编译或 conda-forge**；JIT 缓存必须按架构重新生成
- Windows：jax Windows 支持历史上反复（现由社区/conda 提供）；pinocchio 有 conda 包
- HarmonyOS：免谈

**需提前确认**：
1. JIT 缓存的生成流程是否文档化、可复现（IK `README.md:117-122` 提到缓存随包发布——aarch64/Windows 需要重做）；
2. 冷启动时间在无缓存时各平台是否可接受；
3. **IK server（5200 WS）接口是否足够干净，能把 IK 直接服务化**——若可以，HarmonyOS/弱算力客户端的问题直接消失（B 策略的关键验证点）；
4. 50Hz 双臂在 aarch64（RK3588S 级 A76）上的实测延迟——需求文档说 modest x86 已接近预算。

**验证 spike**：aarch64 Linux 上重建 IK 环境（conda-forge pinocchio + jaxlib），跑通一次 50Hz 求解并记录 p50/p99。

## 4. 系统工具链与设备管理

**现状**：bash launch 脚本、`ip link`/`udevadm`、udev 规则（`99-pico-vr.rules` + plugdev）、root 仅用于 CAN 初始化、信号 trap。

**差异**：Windows 无 udev/ip link/bash；设备枚举靠 COM 口/设备序列号；权限模型完全不同。HarmonyOS 对应物是 hdc + 系统参数（`param set/get`，我们黑客松的 HDC 模式切换就是这么干的）。

**需提前确认**：
1. launch 工具链是否值得重写为 Python launch 或跨平台 runner——先盘点 bash 脚本数量和复杂度；
2. Windows 下多 CAN 适配器的**稳定枚举**（udev  pinning 的等价物）：PCAN 的 channel handle 分配是否确定；
3. adb reverse 在 Windows 的行为（头显链路，需求文档 §6 提到重枚举即断）。

## 5. 相机与 USB

**现状**：3× RealSense（pyrealsense2，848×480@30，JPEG q90 源端压缩），7 个 USB 设备、3 个吃 USB-3 带宽。

**差异**：librealsense 支持 Windows/Linux/macOS（aarch64 官方支持），无 HarmonyOS。

**需提前确认**：
1. Windows 上 pyrealsense2 多相机 30fps 实测；
2. USB 拓扑规划——**我们 3588S 上实测过双相机把 USB2 等时带宽挤爆（usb_submit_urb -28）**，目标平台如果 USB 控制器少，这是物理层 gap，不是软件能救的；
3. HarmonyOS/弱客户端场景：相机流是否走 zenoh 图像通道（7447）转发——已有基础设施，确认带宽和延迟预算。

## 6. VR 桥接

**现状**：Web 页面要求 WebXR `immersive-ar` + `unbounded` reference space + WebGL（`VRDriverPlaneCamera.js`）；目标设备 PICO 4 Ultra / Meta Quest。

**差异**：HarmonyOS ArkWeb 无 WebXR immersive-AR（需求文档 §4 已判定）；Windows/macOS 浏览器正常。

**需提前确认**：HarmonyOS 若要做头显替代，只有原生应用（ArkTS + 渲染自研）一条路——**大概率结论是"不支持"，但要白纸黑字确认后写进兼容性声明**，避免后续反复被问。

## 7. 网络、时钟与分布式

**现状**：DDS 组播发现 + UDP 7400–7500；chrony/PTP 时钟同步是多机**硬要求**（device-onboarding 文档）；CycloneDDS 与 FastDDS 互不发现。

**需提前确认**：
1. Windows 的组播/防火墙行为（企业网络下 DDS 发现常挂）；W32Time 能否满足时钟同步精度，还是必须装 PTP 客户端；
2. 跨 OS 混部时 DDS 实现必须统一（全部 FastDDS 或全部 CycloneDDS）；
3. HarmonyOS 客户端走纯 HTTPS/SSE/WS，无 DDS——确认 8765/5200/8080 三个面板在 ArkWeb 的渲染（需求文档标注 untested）。

## 8. 数据采集与存储

**现状**：452 MB/s HDF5 写入上限实测，~14 GB/h 数据量，h5py。

**需提前确认**：Windows 文件系统（NTFS）持续写入性能与 h5py 行为一致即可，风险低；HarmonyOS 不涉及（B 策略下采集永远在 Linux 节点）。

## 9. 构建、分发与安装

**需提前确认**：
1. 各 OS 的分发形态：Linux 现用 git bundle + 自包含包（README:234-250）；Windows 是 conda 环境 yaml 还是安装器？
2. 版本矩阵 CI：至少 Linux x86_64 / Linux aarch64 / Windows 三线的最小冒烟测试（节点起得来、CAN 假设备、相机假帧）；
3. 依赖锁定：哪些 pip 包有 aarch64/Windows wheel，哪些要源码编译（pinocchio 是头号嫌疑）。

---

## 10. 建议的 spike 顺序（先探后定）

| 序 | Spike | 回答的问题 | 预估 |
|---|---|---|---|
| 1 | Windows + python-can(pcan) 单臂 CAN FD 50Hz 实测 | Gap 1 是否致命 | 1–2 天 |
| 2 | RoboStack 起 rclpy 最小节点 + 真消息类型 | Gap 2 路线成立与否 | 0.5–1 天 |
| 3 | aarch64 重建 IK 栈 + 50Hz 延迟实测 | Gap 3 / 3588S 类平台可行性 | 2–3 天 |
| 4 | IK server 服务化切口评审（5200 WS 接口审一遍） | B 策略是否一剑封喉 | 0.5 天（读代码） |
| 5 | HarmonyOS ArkWeb 打开三个 dashboard | B 策略客户端面确认 | 0.5 天 |
| 6 | Windows 多 RealSense 30fps + USB 拓扑 | Gap 5 | 1 天 |

**做完 Spike 1/2/4 就能回答"Windows 支持是项目还是幻想"；做完 3/4/5 就能回答"HarmonyOS 到底沾不沾边"。**

## 11. 已知结论（不需要再探索，直接采纳）

- macOS：放弃（slcan 路径已死，官方不支持）；
- HarmonyOS 作为计算节点：放弃（无 ROS2、无 SocketCAN、无 x86_64 兼容）；
- HarmonyOS 作为 VR 头显：放弃（无 WebXR immersive-AR）；
- HarmonyOS 作为 dashboard 客户端：大概率可行，Spike 5 确认即可；
- 无 GPU 需求：全平台成立（jax[cpu] 强制）。

# Airspeed Hardware & Computational Requirements (2026-08-14)

Source: evidence scan of `src/`, configs, and `docs/` (file:line citations
throughout). Verdict up front, analysis after.

## Verdict

| Platform | As compute node (any subsystem) | As client device |
|---|---|---|
| **Linux x86_64 (Ubuntu 22.04-class)** | ✅ **Required.** The only supported platform | ✅ |
| **Windows** | ❌ Full stack: no. Sensor/teleop pieces: theoretically portable, not supported today | ⚠️ dashboards only (untested) |
| **macOS** | ❌ No (a vestigial `slcan` branch exists but is dead code) | ⚠️ dashboards only (untested) |
| **HarmonyOS device** | ❌ No | ❌ VR bridge client (needs WebXR immersive-AR — not available). ⚠️ plain dashboards (HTTPS/SSE) should work |

**The irreducible hard floor: Linux x86_64 with kernel SocketCAN, Python
3.10, ROS2 Humble.** Everything else is negotiable; those three are not.

## 1. Mandatory platform requirements

| Requirement | Why | Evidence |
|---|---|---|
| Linux (Ubuntu 22.04-class) | ROS2 Humble binaries; kernel SocketCAN; `ip link`/`udevadm`; FastDDS | `README.md:84-87`, `setup_can.sh:16-72` |
| x86_64 | IK dependency bundle binaries are x86_64 | IK `README.md:58,236-237` |
| Python 3.10 (system) | ROS2 Humble's Python bindings are compiled against system Python 3.10; IK server hardcodes `/opt/ros/humble/lib/python3.10/site-packages` | `README.md:84-87,346-349`; `server/main.py:33` |
| ROS2 Humble at `/opt/ros/humble` | rclpy + sensor/geometry/std msgs; every launch script sources it | launch scripts; `README.md:168` |
| Kernel SocketCAN | arms (CAN FD 1M/5M) and hands (classic CAN 1M) both open `can.interface.Bus(interface="socketcan")` | `damiao.py:160-171`; `socet_can_interface.py:106` |
| Root for CAN setup only | `ip link` + `udevadm` in `setup_can.sh` | `setup_can.sh:16-19` |
| No sudo for runtime nodes | FastDDS shared-memory transport is per-user | gap analysis `docs/...20260812.md:109-118` |
| No GPU needed | `jax[cpu]` everywhere, `JAX_PLATFORMS=cpu` forced | IK `launch/start.sh:50` |

## 2. Hardware bill of materials (current deployment)

| Role | Hardware | Interface |
|---|---|---|
| Arms | 2× OpenArm (Damiao MIT motors: DM8009 shoulders, DM4340/P elbow, DM4310 wrist+gripper), CAN FD | PEAK PCAN-USB Pro FD (`0c72:0011`) → can0/can1 |
| Dexterous hands | 2× OYMotion ROH-LiteS001, classic CAN 1 Mbps | 2× PEAK PCAN-USB (`0c72:000c`) → can2/can3 |
| Cameras | 2× RealSense D405 (wrist) + 1× D435/D455 (head), USB 3.2 | `pyrealsense2`, 848×480 @ 30 fps |
| VR headset | PICO 4 Ultra (`2d40:00b6/00b7`; Meta Quest also supported) | USB + `adb reverse`, or WiFi/HTTPS |
| Operator UI | any browser | HTTPS dashboards :8765, :5200, :8080 |

USB topology note: 3 cameras + 3 CAN adapters + headset = **7 USB devices**,
3 of them USB-3 bandwidth-hungry (cameras) — plan hubs/controllers
accordingly (current deployment uses two hubs, see `lsusb` evidence
2026-08-11).

## 3. Compute budget (measured, from repo data)

| Subsystem | Load | Evidence |
|---|---|---|
| IK solver | JAX CPU (pyroki/jaxlie, dense_cholesky, ≤25 iters), **50 Hz** bimanual with self-collision; JIT cache shipped (~700 MB `.pydeps` + `.cache/jax_compilation`) | `solver_loop.py:21-22`; IK `README.md:117-122` |
| Arm control | **50 Hz** MIT batch commands; gravity pinocchio RNE cached at ~5 Hz (full RNE/14 joints at 50 Hz was too expensive) | `robot.yaml:56`; `arm_controller.py:84-99` |
| ROH hands | 30 Hz state pub, 20 Hz VR control, 50 ms current monitor | `roh_hand.yaml` |
| Cameras | 3× 848×480@30 JPEG q90 at source: encode p50 0.71 ms/frame, 42–65 KB/frame | `docs/camera-image-compression-investigation.md:35-62` |
| Recording | measured ceiling **452 MB/s** HDF5, 3 000+ msg/s, 10 MB images @ 30 Hz | data_collection `README.md:380-389` |
| Data volume | **~14 GB/h** at JPEG q90 (vs ~299 GB/h raw) | camera investigation doc |
| Transport | zenoh side channel ~0.9 ms/frame vs rclpy ~130 ms/MB | `zenoh_side_channel.py:1-4` |

**Suggested minimum node spec** (derived, not measured end-to-end):
x86_64 CPU ≥ 4 modern cores, 8 GB RAM, SSD (sustained writes for episodes),
2× USB-3 controllers. No GPU.

## 4. OS feasibility analysis

### Linux (required)

Hard Linux-only mechanisms: kernel SocketCAN (`socketcan` interface for both
motor families), `ip link`/`udevadm` interface pinning, `/sys/class/net/can*`,
udev rules for the headset, ROS2 Humble at `/opt/ros`, FastDDS per-user
shared memory, bash launch tooling. This is the only supported platform.

### Windows — full stack: no; partial: uncharted

- **Blocker 1 — CAN**: both motor stacks hardcode `interface="socketcan"`.
  PEAK hardware *can* work on Windows via PCAN-Basic (the vendored OHand SDK
  docs mention it), but no first-party code path uses it. Porting = new CAN
  backend for `damiao.py` and `socet_can_interface.py` (~small surface,
  real work).
- **Blocker 2 — ROS2**: Humble exists for Windows, but the repo assumes
  `/opt/ros/humble` paths, system-Python-3.10 binding layout, and bash
  launch scripts (`ip link`, `source`, signal traps) throughout.
- **Not blockers**: jax[cpu], pinocchio, zenoh, aiohttp, pyrealsense2, adb,
  opencv, h5py all have Windows builds. The camera publisher and the VR
  bridge are the most portable pieces; the data-collection core is mostly
  pure Python + rclpy.
- Verdict: a Windows port is a CAN-backend + launch-tooling project, not a
  recompile. Not supported today.

### macOS — no

The only non-SocketCAN path ever written is an `slcan` branch in
`damiao.py:178-183`, documented as "macOS / USB adapters" and confirmed
dead in practice (`docs/motor-register-audit-joint6-lag-20260723.md:124`).
pyrealsense2/jax/zenoh exist for macOS, but ROS2 Humble + CAN + launch
tooling put this out of scope.

### HarmonyOS — no as a node; limited as a client

- **As compute node: impossible today.** HarmonyOS (incl. NEXT) has no ROS2
  Humble runtime, no Linux SocketCAN, no x86_64 Linux binary compatibility
  for the `.pydeps` bundle. Never mentioned in the repo.
- **As VR client: no.** The bridge page requires a WebXR-capable browser
  with `immersive-ar` sessions, `unbounded` reference space, and WebGL
  (`VRDriverPlaneCamera.js` — `navigator.xr.requestSession("immersive-ar")`,
  `requestReferenceSpace("unbounded")`). HarmonyOS's ArkWeb does not provide
  WebXR immersive-AR — the bridge cannot serve it as a headset substitute.
  (The intended VR devices remain PICO 4 Ultra / Meta Quest over adb or
  WiFi.)
- **As a dashboard client: probably yes.** The recording dashboard
  (`:8765`, SSE-pushed state every 200 ms) and the IK monitor are plain
  HTTPS web apps — any modern browser, including a HarmonyOS phone/tablet,
  should render them (untested; self-signed cert acceptance required).

## 5. Network & multi-machine requirements

- Local bus: ROS2/DDS (`ROS_DOMAIN_ID=0`), discovery multicast
  `239.255.0.1:7400/7401` + unicast UDP 7400–7500 + ephemeral 32768–60999;
  firewalled setups need inbound unicast UDP allowed, else topics show but
  no data flows. CycloneDDS and FastDDS do not inter-discover
  (`docs/ros2-multicast-vs-unicast-firewall-20260812.md`).
- Ports: 5100 (VR bridge), 5200 (IK server + WS), 8080 (IK 3D monitor),
  8765 (recording UI), 7447 (zenoh image channel), adb reverse tunnels
  5100/5200 to the headset.
- Cross-subnet: ROS2→JSON→WS relay over SSH tunnel preserving
  `header.stamp`; **chrony/PTP clock sync is a hard requirement** for
  multi-machine stamping (`docs/device-onboarding-schema-guide.md:246-267`).
- Repo distribution to offline machines: `git bundle` over SSH; the IK
  adaptor is a self-contained bundle needing only Python 3.10 + x86_64
  Linux, no network (IK `README.md:234-250`).

## 6. Scaling limits & operational constraints

- One process may own a CAN channel (hands: `HAND_RESP_UNMATCHED_CMD`;
  arms: single batched MIT owner). Never run two stacks on one bus.
- FastDDS shared memory is per-user: all cooperating nodes must run as the
  same (non-root) user.
- Headset link: adb reverse mappings die on device re-enumeration
  (sleep/wake); udev rule `99-pico-vr.rules` + plugdev group required (rule
  documented in gap analysis, not checked into the repo).
- Recording throughput is not the bottleneck (452 MB/s ceiling vs ~14 GB/h
  ≈ 4 MB/s actual); sustained capture is bounded by disk capacity and USB-3
  camera bandwidth first.
- IK adaptor is CPU-bound single-process; 50 Hz bimanual + collision is
  already near its latency budget on modest CPUs (JIT cache mandatory for
  acceptable cold-start).

## 7. References inside the repo

- `README.md` (platform, ROS2 contract, distributed deployment)
- `src/robot_interface/openarm/openarm-ik-ros2-adaptor/README.md` (x86_64
  bundle, JIT cache)
- `docs/camera-image-compression-investigation.md` (camera load numbers)
- `docs/ros2-multicast-vs-unicast-firewall-20260812.md` (DDS networking)
- `docs/device-onboarding-schema-guide.md` (multi-machine, clock sync)
- `docs/post-integration-hardware-gap-analysis-20260812.md` (DDS/sudo, CAN,
  adb lessons)

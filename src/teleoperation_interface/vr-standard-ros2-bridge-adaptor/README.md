# VR-Standard ROS2 Bridge Adaptor

HTTP/HTTPS-to-ROS2 bridge that receives VR device pose and button data and publishes it
as standard `PoseStamped` and `Float32MultiArray` messages matching the
[AIRSPEED teleoperation interface convention](../README.md).

## What It Does

- Runs an HTTP(S) server that a VR device (Meta Quest, Pico, etc.) POSTs JSON pose data to
- Parses the JSON into head / left-hand / right-hand poses and button states
- Publishes to six standard ROS2 topics — the same topics the data collection service
  subscribes to

## Published ROS2 Topics

| Topic | Message Type | Content |
|-------|-------------|---------|
| `/vr/head_pose` | `geometry_msgs/PoseStamped` | Headset position + orientation |
| `/vr/left_pose` | `geometry_msgs/PoseStamped` | Left controller position + orientation |
| `/vr/right_pose` | `geometry_msgs/PoseStamped` | Right controller position + orientation |
| `/vr/left_buttons` | `std_msgs/Float32MultiArray` | Left controller button/trigger values (6 channels) |
| `/vr/right_buttons` | `std_msgs/Float32MultiArray` | Right controller button/trigger values (6 channels) |
| `/vr_raw_data` | `std_msgs/String` | Raw JSON string (debug / backward compat) |

These topics match the [teleoperation interface data convention](../README.md). The data
collection service can subscribe to any subset declared in its session YAML.

## Prerequisites

| Dependency | Check Command | Install |
|-----------|---------------|---------|
| Python 3.10 | `python3.10 --version` | `sudo apt install python3.10` |
| ROS2 Humble | `echo $ROS_DISTRO` | [ROS2 Humble install](https://docs.ros.org/en/humble/Installation.html) |
| aiohttp | `python3.10 -c "import aiohttp"` | `pip install aiohttp` |
| pyOpenSSL | `python3.10 -c "import OpenSSL"` | `pip install pyOpenSSL` |
| ADB (optional) | `adb version` | `sudo apt install adb` |

ROS2 must be sourced before running:

```bash
source /opt/ros/humble/setup.bash
```

## Quick Start

Default config binds to `127.0.0.1:5100` with HTTPS (cert auto-detected from `pem/`).

```bash
cd vr-standard-ros2-bridge-adaptor
source /opt/ros/humble/setup.bash
/usr/bin/python3.10 vr_bridge_server.py
```

## Connecting the VR Device

Two connection methods. USB/ADB is simpler and more reliable. WiFi works when the
device and host are on the same network.

### Method 1: USB + ADB (recommended)

The VR device connects through the USB cable. ADB forwards a local port on the device
to the host machine. The server auto-configures ADB reverse on startup.

**On the host:**

```bash
# HTTP (no certs needed, USB is already encrypted)
/usr/bin/python3.10 vr_bridge_server.py --port 5100 --host 0.0.0.0 --no-ssl

# HTTPS (self-signed cert for 127.0.0.1 included in pem/)
/usr/bin/python3.10 vr_bridge_server.py --port 5100 --host 127.0.0.1
```

**On the VR device:**

1. Connect the VR device to the host via USB cable
2. On the VR device, open the browser and navigate to:
   - HTTP mode: `http://127.0.0.1:5100`
   - HTTPS mode: `https://127.0.0.1:5100`
3. If HTTPS: accept the self-signed certificate warning on first visit
4. The page loads a WebXR session — follow the on-screen prompt to enter VR/immersive mode
5. Once tracking starts, pose and button data flow to ROS2 automatically

**Verify data is flowing:**

```bash
source /opt/ros/humble/setup.bash
ros2 topic hz /vr/right_pose       # should show ~60 Hz
ros2 topic echo /vr/right_pose --once
```

### Method 2: WiFi / LAN

The VR device connects over the local network. No USB cable required. The server must
bind to the machine's LAN IP and the device must be on the same WiFi.

**On the host:**

```bash
# Install pyOpenSSL for cert generation (one time)
pip install pyOpenSSL

# Replace <host-ip> with your machine's actual IP
/usr/bin/python3.10 vr_bridge_server.py --port 5100 --host <host-ip>
```

**On the VR device:**

1. Ensure the VR device is on the same WiFi network as the host
2. Open the browser and navigate to `https://<host-ip>:5100`
3. Accept the self-signed certificate warning
4. Enter VR/immersive mode when prompted

### Prerequisite: VR Device Developer Mode

Both methods require developer mode enabled on the VR device:

- **Meta Quest**: Phone app → Devices → Developer Mode (toggle on)
- **Pico**: Settings → About → tap Build Number 7 times → Settings → Developer Options

## Configuration

Edit `config/config.json`:

| Key | Default | Description |
|-----|---------|-------------|
| `port` | 5100 | Server port |
| `ip` | 127.0.0.1 | Bind address and certificate IP |
| `vr_data_rate` | 60 | Expected VR data rate in Hz (informational) |
| `source_type` | vr | Device type identifier |
| `camera_width` | 640 | Reserved for camera streaming |
| `camera_height` | 480 | Reserved for camera streaming |
| `share_memory_name` | "" | Reserved for shared memory path |

A self-signed certificate for `127.0.0.1` is included in `pem/`. When binding to a
different IP, the server auto-generates a new certificate on first run (requires
`pyOpenSSL`).

### CLI Flags

| Flag | Default | Description |
|------|---------|-------------|
| `--port` | from config (5100) | Server port |
| `--host` | from config (127.0.0.1) | Bind address |
| `--no-ssl` | off | Serve plain HTTP instead of HTTPS |
| `--no-adb` | off | Skip ADB reverse port forwarding |

CLI flags override config file values.

## JSON Data Format

The VR web UI POSTs JSON to `/poseData` at the configured rate (~60 Hz):

```json
{
  "state": "NORMAL",
  "battery": 0.8,
  "head": {
    "position": {"x": 0.0, "y": 1.6, "z": 0.1},
    "rotation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  },
  "left": {
    "position": {"x": -0.2, "y": 0.8, "z": -0.3},
    "rotation": {"x": 0.0, "y": 0.6, "z": -0.3, "w": 0.7},
    "button": [
      {"value": 0.0},
      {"value": 0.0},
      {"value": 0.0},
      {"value": 0.0},
      {"value": 0.0},
      {"value": 0.0}
    ]
  },
  "right": {
    "position": {"x": 0.2, "y": 0.8, "z": -0.3},
    "rotation": {"x": 0.2, "y": 0.1, "z": 0.4, "w": 0.9},
    "button": [
      {"value": 0.0},
      {"value": 0.0},
      {"value": 1.0},
      {"value": 0.0},
      {"value": 0.0},
      {"value": 0.5}
    ]
  }
}
```

Button channels are 0.0–1.0 (float). Channel order is device-specific; the bridge passes
values through unchanged.

### Coordinate Frame

The VR device's native frame (Y-up, right-handed) is passed through as-is. Position is
in meters, orientation is a normalized quaternion (xyzw). If your application expects
ROS standard coordinates (X-forward, Y-left, Z-up), apply the transform downstream or
in the session YAML field mapping.

## Deploying to Another Machine

This directory is a self-contained deployable bundle:

```bash
# On source machine — create an archive
cd airspeed-main-v1.0/src/teleoperation_interface
tar -czf vr-bridge-adaptor.tar.gz vr-standard-ros2-bridge-adaptor/

# On target machine — extract and run
tar -xzf vr-bridge-adaptor.tar.gz
cd vr-standard-ros2-bridge-adaptor
source /opt/ros/humble/setup.bash
/usr/bin/python3.10 vr_bridge_server.py
```

The target machine needs the prerequisites above. No `pip install` or network
access beyond OS packages.

## Files

```
vr-standard-ros2-bridge-adaptor/
├── vr_bridge_server.py       # Main server entry point
├── config/
│   └── config.json           # Port, IP, data rate, camera settings
├── static/                   # VR web UI (served to device browser)
│   ├── VRDriverPlaneCamera.html
│   ├── VRDriverPlaneCamera.js
│   ├── VRDriverPlaneCamera.css
│   └── ...                   # Fonts, icons, logos
├── pem/                      # SSL certificate (self-signed, for 127.0.0.1)
│   ├── cert.pem
│   ├── key.pem
│   └── IP_address.txt
└── README.md                 # This file
```

## Session YAML (Data Collection Service)

To record these topics with the data collection service, add to your session YAML:

```yaml
streams:
  - name: "right_pose"
    source: teleop
    topic: "/vr/right_pose"
    message_type: "geometry_msgs/PoseStamped"
    time_domain: ros_header
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 10
    fields:
      - path: "pose.position.x"
        type: float64
        required: true
      - path: "pose.position.y"
        type: float64
        required: true
      - path: "pose.position.z"
        type: float64
        required: true
      - path: "pose.orientation.x"
        type: float64
        required: true
      - path: "pose.orientation.y"
        type: float64
        required: true
      - path: "pose.orientation.z"
        type: float64
        required: true
      - path: "pose.orientation.w"
        type: float64
        required: true

  - name: "right_buttons"
    source: teleop
    topic: "/vr/right_buttons"
    message_type: "std_msgs/Float32MultiArray"
    time_domain: ros_receive
    qos:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10
    fields:
      - path: "data"
        type: sequence
        required: true
```

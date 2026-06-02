# Camera Stream

RealSense camera Image + CameraInfo publishing to ROS2. Streams at the camera's
native rate via a background thread — no artificial frame rate cap.

Deploy independently of arm controllers. Add cameras and stream types by editing
`config/camera.yaml` only — zero code changes.

## Published ROS2 Topics

Topics are derived from the config. Default three-camera setup:

| Topic | Type | Content |
|-------|------|---------|
| `/camera/right_wrist/image_raw` | `sensor_msgs/Image` | JPEG-encoded, native rate (~30 Hz single / ~15 Hz multi-stream) |
| `/camera/right_wrist/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (latched, TRANSIENT_LOCAL) |
| `/camera/head/image_raw` | `sensor_msgs/Image` | JPEG-encoded |
| `/camera/head/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (latched) |
| `/camera/left_wrist/image_raw` | `sensor_msgs/Image` | JPEG-encoded |
| `/camera/left_wrist/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics (latched) |

CameraInfo is published once at startup with TRANSIENT_LOCAL durability (latched) —
late subscribers receive intrinsics without waiting for a republish.

## Quick Start

```bash
cd camera-stream-adaptor
source /opt/ros/humble/setup.bash
python3 camera_publisher.py

# Custom config path
python3 camera_publisher.py --config-dir /path/to/config
```

## Configuration

All in `config/camera.yaml`. Per-camera stream configuration — each camera
declares which streams (color, depth, infra) to publish. Streams not listed
under a camera are not opened for that camera.

### Default Config

```yaml
cameras:
  right_wrist:
    serial: "auto"
    topic: "/camera/right_wrist"
    frame_id: "camera_right_wrist_optical_frame"
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        encoding: "jpeg"
        jpeg_quality: 70

  head:
    serial: "auto"
    topic: "/camera/head"
    frame_id: "camera_head_optical_frame"
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        encoding: "jpeg"
        jpeg_quality: 70

  left_wrist:
    serial: "auto"
    topic: "/camera/left_wrist"
    frame_id: "camera_left_wrist_optical_frame"
    streams:
      color:
        enabled: true
        width: 640
        height: 480
        fps: 30
        encoding: "jpeg"
        jpeg_quality: 70

max_cameras: 3

intrinsics:
  right_wrist: { ... }
  head: { ... }
  left_wrist: { ... }
```

### Adding a Camera

Add an entry under `cameras:` and intrinsics under `intrinsics:`:

```yaml
cameras:
  my_new_camera:
    serial: "auto"                 # "auto" or specific serial number
    topic: "/camera/my_new_camera"
    frame_id: "my_camera_optical_frame"
    streams:
      color:
        enabled: true
        width: 1280
        height: 720
        fps: 30
        encoding: "jpeg"
        jpeg_quality: 80

intrinsics:
  my_new_camera:
    width: 1280
    height: 720
    fx: 920.0
    fy: 920.0
    cx: 640.0
    cy: 360.0
    distortion_model: "plumb_bob"
    distortion_coeffs: [0.0, 0.0, 0.0, 0.0, 0.0]
```

### Stream Types

| Stream | Supported | Notes |
|--------|-----------|-------|
| `color` | Yes (via lerobot SDK) | RGB frame, JPEG-encoded at source |
| `depth` | Not yet | Requires pyrealsense2 direct access |
| `infra_left` | Not yet | Requires pyrealsense2 direct access |
| `infra_right` | Not yet | Requires pyrealsense2 direct access |

If a stream type is configured but the backend doesn't support it, a `[WARN]`
is printed at startup and the stream is skipped — the node does not crash.

### Camera Discovery

Cameras are matched to config entries by **discovery order** — the first
discovered camera gets the first config entry, the second gets the second, etc.
Use `serial: "<number>"` to pin a specific physical camera to a config entry.

If a config entry can't be matched to any discovered camera, a `[WARN]` is
printed and that entry is skipped.

### Rate

Frames stream at the camera's native rate — no `create_timer` cap. A single
color stream runs at ~30 Hz. With multiple streams enabled per camera, the
composite rate drops to ~15 Hz (USB 3.2 bandwidth limit). The console prints
the actual achieved rate every 10 seconds.

## Session YAML (Data Collection Service)

```yaml
streams:
  - name: "right_wrist_camera"
    source: sensor
    topic: "/camera/right_wrist/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: jpeg
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 5
    fields:
      - path: "data"
        type: bytes
        required: true

  - name: "head_camera"
    source: sensor
    topic: "/camera/head/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: jpeg
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 5
    fields:
      - path: "data"
        type: bytes
        required: true

  - name: "left_wrist_camera"
    source: sensor
    topic: "/camera/left_wrist/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: jpeg
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 5
    fields:
      - path: "data"
        type: bytes
        required: true
```

## Reference

- [Sensor Interface Convention](../README.md) — Image + CameraInfo spec
- [Adapter Contract Guide](../../../memodocs/adapter_contract_guide.md) — image_encoding rules
- [Data Collection Service](../../data_collection_service/README.md) — session YAML format

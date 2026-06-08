# Sensor Interface — Data Convention

This document defines the data conventions that **any camera, depth sensor, or
environmental sensor** must follow for the data collection service to correctly record
its output.

Write a ROS2 publisher that reads your sensor SDK and publishes messages matching these
conventions. The data collection service subscribes to whatever topics the session YAML
declares — it has no baked-in knowledge of specific sensors.

> **The bundled adaptor is an example.** The RealSense camera publisher is a reference
> implementation. To use a different camera (Luxonis, Orbbec, USB webcam, thermal,
> LiDAR) or add an IMU, create a new adaptor folder with a `launch/start.sh` script
> and a ROS2 publisher node. The interface only cares about the topic contract
> below — any sensor that publishes `Image`, `CameraInfo`, `Imu`, or `PointCloud2`
> is compatible.

## Launch

```bash
cd sensor_interface
bash run_global_config.sh                    # starts the configured adaptor
```

`global_config.yaml` points to the active adaptor. `run_global_config.sh` reads it
and launches the adaptor. Uses `python3` from PATH — activate your env before running.

```yaml
# global_config.yaml
adaptor: "camera-stream-adaptor"
# Uses python3 from PATH — activate your env before running
```

To add a new camera or sensor:

1. Create `<your-adaptor>/` with a `launch/start.sh` script
2. Change `global_config.yaml` → `adaptor: "your-adaptor"`
3. Uses `python3` from PATH — activate your env before running
4. Run `bash run_global_config.sh`

## Two-Layer Configuration

The project separates what gets recorded from how hardware is configured. Two YAML files,
two responsibilities, no source code edits.

| Layer | Location | Owned By | What It Controls |
|-------|----------|----------|-----------------|
| Session YAML | `data_collection_service/config/` | Data collection service | Topics to subscribe to, message types, field contracts, recording control mode |
| Interface YAML | `<adaptor>/config/` (each adaptor owns its config) | Sensor driver | Device selection, stream resolution, encoding, frame rate, calibration |

### What the Session YAML Controls (Data Pipeline)

Which ROS2 topics to record, what message types they carry, which fields to extract,
QoS settings, and the `image_encoding` directive (store as JPEG or raw). This file is
hardware-agnostic — it declares "subscribe to `/camera/color/image_raw` of type `Image`"
without knowing what camera produces that topic.

### What the Interface YAML Controls (Hardware Hyperparameters)

Device-specific settings that change when you swap hardware but the data pipeline stays
the same:

```yaml
# <your-adaptor>/config/my_camera.yaml
camera:
  type: "generic_rgbd"           # SDK driver to use
  serial_number: "241322301478"  # which physical device (or "auto")
streams:
  color:
    enabled: true
    width: 640
    height: 480
    fps: 30
    encoding: "jpeg"             # hardware encoding to use
    topic: "/camera/color/image_raw"
  depth:
    enabled: true
    width: 640
    height: 480
    fps: 30
    encoding: "16UC1"
intrinsics:
  fx: 615.0                      # or "auto" from SDK
  fy: 615.0
  cx: 320.0
  cy: 240.0
tf:
  publish_tf: true
  camera_frame: "camera_link"
  parent_frame: "base_link"
```

Edit this file to match your camera setup — no source code changes needed.

### Why Two Layers

The session YAML answers "what data do I want to record?" The interface YAML answers
"how do I talk to this specific sensor?" When you swap one camera for another (or add a
second one), only the interface YAML changes. The session YAML and the data collection
pipeline stay the same.

## What the Data Collection Service Expects

The collector records sensor data as:

| Category | Required Message Type | Storage Format |
|----------|----------------------|----------------|
| Image (camera, depth, thermal, etc.) | `sensor_msgs/Image` | AIRS image group — variable-length byte sequences |
| Numeric sensor array | `std_msgs/Float32MultiArray` | AIRS vector group — N dimensions |

Additional message types (`Imu`, `PointCloud2`, `LaserScan`) can be added by
creating a new adapter binding — see the adapter contract guide in `memodocs/`.

The session YAML declares which topics to subscribe to — declare only the ones your
sensor provides.

## Image Convention

`sensor_msgs/Image` is the universal ROS2 message for 2D array data. It handles color
cameras, depth sensors, infrared, thermal, and any other pixel-based modality through
its `encoding` field.

```python
from sensor_msgs.msg import Image

msg = Image()
msg.header.stamp = frame.hardware_timestamp      # exposure time from sensor
msg.header.frame_id = "sensor_frame"              # TF frame of this sensor
msg.height = 480
msg.width = 640
msg.encoding = "jpeg"                             # tells consumers how to decode
msg.is_bigendian = 0
msg.step = len(image_bytes)                       # bytes per row (raw) or total bytes (encoded)
msg.data = image_bytes
```

### Encoding Field

The `encoding` field is the key to interoperability. It tells every consumer how to
interpret the byte array:

| Encoding | Modality | Bytes per Pixel | When to Use |
|----------|----------|----------------|-------------|
| `jpeg` | Color, any resolution | Variable (~200 KB at 640×480) | **Recommended for color.** Smallest message size, stored as-is. |
| `rgb8` | Color (red-green-blue) | 3 | When you need raw pixel access |
| `rgba8` | Color with alpha | 4 | When transparency matters |
| `bgr8` | Color (OpenCV native) | 3 | When your SDK outputs OpenCV format |
| `16UC1` | Depth (mono 16-bit) | 2 | **Recommended for depth.** Values in millimeters. |
| `mono8` | Grayscale / infrared | 1 | IR feeds, grayscale cameras |
| `mono16` | High-depth grayscale | 2 | Raw sensor values, custom range |
| `8UC3` | Raw color (no color space) | 3 | Avoid — prefer rgb8 or jpeg |

The data collection service stores JPEG-encoded images as-is (zero re-encoding). Other
encodings are re-encoded to JPEG at the writer unless `image_encoding: raw` is set in
the session YAML.

### JPEG Is Recommended for Color

A 640×480 raw RGB frame is ~900 KB. The same frame as JPEG is ~200 KB. Smaller messages
mean less DDS bandwidth, smaller HDF5 files, and faster writes. If your camera SDK
supports hardware JPEG encoding, use it.

### CameraInfo Convention

`sensor_msgs/CameraInfo` carries the intrinsic calibration that downstream consumers
need to project pixels into 3D space.

```python
from sensor_msgs.msg import CameraInfo

msg = CameraInfo()
msg.header.stamp = acquisition_timestamp
msg.header.frame_id = "sensor_frame"
msg.height = 480
msg.width = 640
msg.distortion_model = "plumb_bob"                # or "rational_polynomial", "equidistant"
msg.d = [k1, k2, t1, t2, k3]                      # distortion coefficients
msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0] # intrinsic matrix (row-major, 3×3)
msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0] # rectification (identity if none)
msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0] # projection matrix
```

Publish CameraInfo at least once per sensor, using TRANSIENT_LOCAL durability so late
subscribers (including the collector) receive it without waiting for a republish.

### Image Timestamps

`header.stamp` must be the **exposure time** — when the sensor captured the photons.
Not when your code encoded the JPEG, not when ROS2 dispatched the callback.

```python
# Correct — hardware timestamp
msg.header.stamp = frame.get_timestamp()

# Wrong — records encoding latency, not exposure time
msg.header.stamp = self.get_clock().now().to_msg()
```

If your sensor SDK does not expose hardware timestamps, set `time_domain: ros_receive`
in the session YAML for that stream.

## Topic Naming Convention

Follow REP-2001 conventions. The collector has no fixed topic names.

```
/camera/color/image_raw       # color camera stream
/camera/color/camera_info     # color camera intrinsics
/camera/depth/image_raw       # depth stream
/camera/infra/image_raw       # infrared stream
/imu                          # inertial measurement unit
/scan                         # laser scan
/camera/depth/points          # point cloud
```

Additional message types follow the same REP-2001 convention when
adapter bindings are added.

## QoS Convention

| Data Type | Reliability | Durability | History | Depth |
|-----------|------------|------------|---------|-------|
| Image stream | BEST_EFFORT | VOLATILE | KEEP_LAST | 1 |
| CameraInfo | RELIABLE | TRANSIENT_LOCAL | KEEP_LAST | 1 |

Images use BEST_EFFORT — a dropped frame is better than a delayed one. CameraInfo uses
TRANSIENT_LOCAL so late subscribers receive intrinsics immediately.

## Session YAML Examples

### Color camera with JPEG output

```yaml
streams:
  - name: "color_image"
    source: sensor
    topic: "/camera/color/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: jpeg
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: bytes
        required: true
```

### Depth sensor (16-bit millimeters)

```yaml
streams:
  - name: "depth_image"
    source: sensor
    topic: "/camera/depth/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: raw          # store 16-bit values without re-encoding to JPEG
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: bytes
        required: true
```

### Infrared / thermal camera

```yaml
streams:
  - name: "infra_image"
    source: sensor
    topic: "/camera/infra/image_raw"
    message_type: "sensor_msgs/Image"
    time_domain: ros_header
    image_encoding: raw
    qos:
      reliability: best_effort
      durability: volatile
      history: keep_last
      depth: 1
    fields:
      - path: "data"
        type: bytes
        required: true
```

## Reference

- [Data Collection Service](../data_collection_service/README.md) — core architecture, session YAML format
- [Camera Stream Pipeline Mapping](PIPELINE_MAPPING.md) — 5-hop trace from camera discovery to ROS2 Image publish

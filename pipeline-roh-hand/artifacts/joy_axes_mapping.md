# VR Joy Axes Mapping — verified from source

Evidence: `src/teleoperation_interface/vr-standard-ros2-bridge-adaptor/vr_bridge_server.py` and `openarm/openarm-ik-ros2-adaptor/server/vr_normalizer.py` (step 01 gates, this repo).

The canonical bridge builds `Joy.axes` from the VR client's `button` array
(`buttons[:6]`, zero-padded) — byte-identical construction to the old 115
`Float32MultiArray.data` version, so channel order is preserved 1:1.

| axes index | meaning | used by ROH hand controller for |
|---|---|---|
| 0 | trigger (analog 0..1) | trigger analog → grasp interpolation |
| 1 | grip / middle finger | — |
| 2 | reserved | — |
| 3 | thumbstick click | thumbstick click → gesture mode cycle |
| 4 | A button | A (right) → clear open latch |
| 5 | B button | B (right) → open / reset hand, set latch |

Pressed threshold: value > 0.5. Trigger clamped to [0, 1].
Conclusion: partner's ROH button semantics port unchanged onto Joy axes
(0=trigger, 3=click, 4=A, 5=B). No index mismatch found.

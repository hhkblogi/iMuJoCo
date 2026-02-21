# TODO

## Caffeine Mode Verification

Caffeine Mode (Off/Half/Full) needs thorough testing on a real device:

- **Off** — simulations should fully pause when switching apps or locking screen
- **Half** — screen stays on; simulations should pause on app switch / screen lock
- **Full** — screen stays on; simulations keep running through screen lock
- Verify transitions between modes take effect immediately
- Verify silent background audio starts/stops correctly in Full mode
- Verify no interference with other audio playback (`.mixWithOthers`)

## Clock Synchronization (Future Work)

The current timestamp-paced replay uses **delta-based timing**: only the difference
between consecutive host timestamps matters, so no clock synchronization between
host and device is required.

Future improvements:

- **Absolute timestamp alignment** would require NTP or PTP-style clock sync
  between host and device. This would enable correlating host-side events
  (e.g., sensor readings) with simulation time.

- **Clock drift compensation** for very long sessions (hours+). The current
  approach assumes host and device clocks tick at the same rate. Over long
  durations, drift between `steady_clock` instances could cause the replay
  cadence to gradually shift. A periodic re-anchoring or drift estimation
  algorithm could mitigate this.

## WebRTC Transport (Future Work)

MJPEG-over-HTTP has 1-3s perceived latency due to client-side buffering in
VLC/ffplay. The sender pipeline is ~1.2ms; the bottleneck is entirely on
the receiver side.

- **WebRTC** would give ~50-150ms end-to-end latency with built-in jitter
  buffer tuning, NAT traversal, and adaptive bitrate
- Could use Apple's VideoToolbox for hardware H.264/HEVC encode (vs
  CoreGraphics JPEG) for better compression and lower CPU usage
- Signaling via a lightweight WebSocket server on the device
- Browser-native playback — no VLC/ffplay needed
- Libraries to evaluate: libwebrtc (Google), or libdatachannel (lighter)

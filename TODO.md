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

## RTP/RTSP Transport — HEVC Pipeline

The RTP/RTSP transport uses **H.265/HEVC** with VideoToolbox hardware encoding and
RFC 7798 RTP payload format. Zero-copy pipeline from GPU render to encoder input:

- Metal renders BGRA directly into `.storageModeShared` MTLBuffer
- `CVPixelBufferCreateWithBytes` wraps the UMA pointer (no pixel copy)
- VideoToolbox HW encoder reads the buffer directly
- Output is HVCC format (4-byte length-prefixed NALUs) into a pre-allocated buffer
- RTP transport parses HVCC lengths inline (no Annex B conversion)

Play with: `ffplay -fflags nobuffer rtsp://<device-ip>:8554/camera0`

## QUIC Transport — Transmission Latency Measurement (Future Work)

The QUIC receiver currently tracks decode latency but not network transmission
latency. `QUICFrameHeader.simulationTime` is MuJoCo model time, not wall-clock.

Options:

- **Wall-clock timestamp in frame header** — server writes
  `ProcessInfo.processInfo.systemUptime` (or `mach_absolute_time`); receiver
  computes `receiveTime - serverTimestamp`. Requires NTP-level clock sync
  (works well on same machine or LAN with NTP).
- **Round-trip ping** — client embeds a timestamp in a lightweight ping message,
  server echoes it back, client computes RTT/2. No clock sync needed but only
  measures round-trip, not one-way.

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

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

## RTP/RTSP Transport — Huffman Table Incompatibility (Known Issue)

The RTP/RTSP transport (RFC 2435 JPEG payload) produces **color corruption** (green/pink
flickering) in VLC and ffplay. Root cause:

- **RFC 2435** mandates that receivers reconstruct JPEG headers using the **standard
  Annex K Huffman tables**. Only quantization tables are transmitted in-band.
- **CGImageDestination** (CoreGraphics JPEG encoder) uses **non-standard Huffman tables**
  that differ from Annex K. The entropy-coded scan data is encoded against these
  custom tables.
- When the receiver rebuilds the JPEG with standard Huffman tables, every Huffman
  code is decoded incorrectly → `bad vlc: 0:0` errors in ffplay, visual corruption.

This is a fundamental mismatch: the scan data is encoded with one set of Huffman tables
but decoded with another. MJPEG/HTTP works because it sends the complete JFIF file
(including the actual Huffman tables used for encoding).

Confirmed by source code review of all major receivers:

- **live555** (`JPEGVideoRTPSource.cpp`): hardcodes Annex K tables, no custom table support
- **FFmpeg** (`rtpdec_jpeg.c`): hardcodes Annex K tables, no alternative path
- **FFmpeg RTP JPEG encoder** (`rtpenc_jpeg.c`): `memcmp`s input Huffman tables against
  Annex K and **refuses to send** if they don't match

Recommended fix: **Replace CGImageDestination with libjpeg-turbo** for the RTP encode
path. libjpeg-turbo defaults to standard Annex K Huffman tables and has NEON SIMD
acceleration on ARM. CGImageDestination continues to work fine for MJPEG/HTTP (which
sends the complete JFIF including custom DHT markers).

Other options:

- **VideoToolbox H.264** — switch to H.264 RTP payload (RFC 6184) with hardware
  encoding. Better compression, no Huffman table issue, but requires different SDP/RTSP
  negotiation and is a larger change
- **Re-encode**: decompress CGImageDestination output, re-compress with libjpeg — wasteful

Current status: **MJPEG/HTTP is the recommended transport**. RTP/RTSP is available in
Settings but produces visual artifacts with standard receivers.

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

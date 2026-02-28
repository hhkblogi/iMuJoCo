# Changelog

All notable changes to iMuJoCo are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/), and this project uses [Conventional Commits](https://www.conventionalcommits.org/).

## [0.1.2] - 2026-02-26

### Added
- gRPC simulation control server with Protobuf definitions (#85)
- gRPC 1.78.0 + Protobuf third-party deps (#80)
- Video streaming: HEVC/RTSP + MJPEG/HTTP with zero-copy GPU pipeline (#74, #75, #76)
- Metal offscreen capture for video streaming (#75)
- Unified RTP/RTSP on same port, HEVC receiver app (#81)
- Unitree G1 (RL) 12-DOF model from unitree_rl_gym (#78)
- Network security: bind restriction, IP picker, port config (#89)
- macOS polish: menu bar, Settings, trackpad controls (#83)
- Performance stats bar (#72)
- Loading spinner while model loads (#71)

### Changed
- Renamed SpscQueue to SpmcQueue (#73)

### Fixed
- iPad/iPhone UI polish: model picker, IP picker, default unlocked (#90)
- Replace audio background mode with BGProcessingTask (#87)
- Eye/lock button UX + per-instance streaming toggle (#77)
- Provisioning profile config and security hardening (#70)
- Guard `insetGrouped` list style to iOS only (#69)

### Performance
- Zero-copy QUIC receiver data flow (#79)

## [0.1.1] - 2026-02-22

### Fixed
- Replace audio background mode with BGProcessingTask for proper iOS background execution

## [0.1.0] - 2026-02-19

Initial release.

### Added
- Real-time MuJoCo physics simulation on iOS, iPadOS, tvOS, macOS
- Metal GPU rendering with Blinn-Phong shading
- Swift-C++ interop (no bridging headers)
- C++ UDP driver library with Python bindings (pybind11)
- FlatBuffers serialization for control/state packets
- Bazel 9 build system with bzlmod
- Multi-simulation grid layout
- Scene lighting, mesh rendering, procedural ground plane
- XYZ orientation axes gizmo
- Settings: Caffeine Mode, Default Lock State, triple-click action
- Bundled models: Humanoid, Simple Pendulum, Car, Unitree G1, Unitree H1, Cassie, ANYmal C
- Instanced rendering + dynamic buffer rightsizing
- App Store submission setup

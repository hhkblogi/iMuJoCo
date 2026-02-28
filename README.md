# iMuJoCo

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-iOS%20%7C%20iPadOS%20%7C%20tvOS%20%7C%20macOS-lightgrey.svg)](https://developer.apple.com)
[![Bazel](https://img.shields.io/badge/Bazel-9-green.svg)](https://bazel.build)

Real-time MuJoCo simulation on Apple devices for your robotics projects.

Control simulations from Python or C++ over UDP/gRPC while Metal renders at 60 fps on-device.

## Overview

iMuJoCo brings the [MuJoCo](https://github.com/google-deepmind/mujoco) physics simulation engine to Apple platforms with native Swift UI and Metal rendering.

- **Swift + Metal** for UI and rendering (no OpenGL)
- **C/C++** for core simulation runtime
- **Swift-C++ interop** for direct calls without bridging overhead
- **UDP + gRPC** for external driver control
- **HEVC/RTSP + MJPEG** video streaming from device

## Project Structure

```
iMuJoCo/
├── imujoco/
│   ├── app/              # SwiftUI application (iOS, iPadOS, tvOS, macOS)
│   ├── core/             # C++ physics runtime + Swift interop
│   ├── render/           # Metal GPU renderer (Blinn-Phong shaders)
│   ├── grpc/             # gRPC simulation control server
│   └── video/            # Video streaming (HEVC/RTSP, MJPEG/HTTP)
│
├── driver/               # External driver: C++, Python (pybind11), Swift
├── schema/               # FlatBuffers (.fbs) + Protobuf (.proto) schemas
├── models/               # MuJoCo XML model files
├── benchmarks/           # Performance benchmarks
├── third_party/          # Vendored C deps + Bazel 9 patches
│
├── MODULE.bazel          # Bazel module config (bzlmod)
├── extensions.bzl        # Module extension for C deps
└── .bazelrc              # Build flags + platform configs
```

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                        app (SwiftUI)                         │
│                  UI, Views, User Interaction                  │
└──────────────────────────────────────────────────────────────┘
                     │                    │
                     ▼                    ▼
┌──────────────────────────┐  ┌────────────────────────────────┐
│    render (Metal GPU)    │  │      video (Metal capture)     │
│  Blinn-Phong, instanced  │  │   HEVC/RTSP, MJPEG/HTTP       │
└──────────────────────────┘  └────────────────────────────────┘
                     │                    │
                     ▼                    ▼
┌──────────────────────────────────────────────────────────────┐
│                   core (Static Library)                       │
│       Swift ←→ C++ Interop, Physics Runtime, Network IO      │
└──────────────────────────────────────────────────────────────┘
          │                │                 │
          ▼                ▼                 ▼
┌──────────────┐  ┌────────────────┐  ┌────────────────────────┐
│    MuJoCo    │  │  UDP (FlatBuf) │  │    gRPC (Protobuf)     │
│   Physics    │  │    ◄── driver  │  │     ◄── client         │
└──────────────┘  └────────────────┘  └────────────────────────┘
```

## Supported Platforms

| Platform | Min Version | Architecture |
|----------|-------------|--------------|
| iOS      | 26.0        | arm64        |
| iPadOS   | 26.0        | arm64        |
| tvOS     | 26.0        | arm64        |
| macOS    | 26.0        | arm64        |

## Getting Started

### Prerequisites

- [Bazel 9.0+](https://bazel.build/install) (or [Bazelisk](https://github.com/bazelbuild/bazelisk))
- Xcode 16.0+
- macOS 15.0+ (for development)

### Clone

```bash
git clone https://github.com/hhkblogi/iMuJoCo.git
cd iMuJoCo
git config core.hooksPath .githooks
```

### Build

**Build the iOS app:**

```bash
bazel build //imujoco/app:app_ios
```

**Run driver tests:**

```bash
bazel test //driver:driver_test
```

**Build individual components:**

```bash
bazel build @mujoco//:mujoco           # MuJoCo library
bazel build //schema:core_schemas       # FlatBuffers codegen
bazel build //imujoco/core              # Core framework
bazel build //imujoco/render            # Render framework
```

### Xcode Development

Generate an Xcode project for development and debugging:

```bash
bazel run //:xcodeproj
open imujoco.xcodeproj
```

Build, run, and debug normally from Xcode.

#### Device Deployment

To deploy to a physical iOS device, set up your Apple Developer Team ID:

```bash
cp imujoco/app/team_config.bzl.template imujoco/app/team_config.bzl
# Edit team_config.bzl and set TEAM_ID to your Apple Developer Team ID
bazel run //:xcodeproj
```

Then select your device in Xcode and build. Xcode handles automatic code signing.

### Platform Configs

Build for different platforms using named configs:

```bash
bazel build //imujoco/app:app_ios --config=ios_device      # iOS device
bazel build //imujoco/app:app_ios --config=ios_sim          # iOS simulator
bazel build //imujoco/app:app_macos --config=macos           # macOS
bazel build //imujoco/app:app_tvos --config=tvos_device     # tvOS device
```

## Driver

The C++ / Python driver library lets you control simulations running on-device from your development machine over UDP. Send joint commands, read sensor state, and replay trajectories.

See [driver/README.md](driver/README.md) for setup and usage.

## gRPC Control

iMuJoCo also exposes a gRPC server for simulation control. The service definition is in [`schema/simulation_control.proto`](schema/simulation_control.proto). This provides a structured RPC alternative to the raw UDP driver for clients that prefer gRPC.

## Troubleshooting

**Bazel build cache stale after switching branches:**
```bash
bazel clean --expunge
bazel build //...
```

**Xcode project out of date:**
Regenerate after any `BUILD.bazel` or `MODULE.bazel` change:
```bash
bazel run //:xcodeproj
```

**`team_config.bzl` missing:**
Required for device deployment only. Copy the template:
```bash
cp imujoco/app/team_config.bzl.template imujoco/app/team_config.bzl
```

**FlatBuffers / Protobuf codegen errors:**
Ensure schemas build first:
```bash
bazel build //schema:core_schemas
```

**Bazel version mismatch:**
This project requires Bazel 9. Use [Bazelisk](https://github.com/bazelbuild/bazelisk) for automatic version management.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

Apache License 2.0 - see [LICENSE](LICENSE) for details.

### Third-Party Licenses

- [MuJoCo](https://github.com/google-deepmind/mujoco) - Apache License 2.0

### Bundled Models

- Car — [MuJoCo](https://github.com/google-deepmind/mujoco) · Apache 2.0
- Humanoid (Supine) — [MuJoCo](https://github.com/google-deepmind/mujoco) · Apache 2.0
- Simple Pendulum — iMuJoCo · Apache 2.0
- Agility Cassie — [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) · BSD-3
- Unitree G1 — [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) · BSD-3
- Unitree H1 — [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) · BSD-3

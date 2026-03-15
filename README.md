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
| macOS    | 15.0        | arm64        |

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

To deploy to a physical iOS/iPadOS device, set up code signing:

1. **Create `team_config.bzl`** from the template:

   ```bash
   cp imujoco/app/team_config.bzl.template imujoco/app/team_config.bzl
   ```

2. **Set your Apple Developer Team ID.** Find it by running:

   ```bash
   security find-certificate -c "Apple Development" -p | openssl x509 -noout -subject -nameopt multiline | grep organizationalUnitName
   ```

   Edit `imujoco/app/team_config.bzl` and set `TEAM_ID`:

   ```python
   TEAM_ID = "YOUR_TEAM_ID_HERE"
   ```

3. **Set the development provisioning profile name.** Xcode creates per-bundle-ID
   profiles automatically when you first build for a device. The profile name is
   typically `iOS Team Provisioning Profile: <bundle-id>`. Set `DEV_PROFILE_NAME`
   to match:

   ```python
   DEV_PROFILE_NAME = "iOS Team Provisioning Profile: com.hhkblogi.imujoco.app"
   ```

   If you're unsure which profiles exist on your machine, list them:

   ```bash
   find ~/Library/Developer/Xcode/UserData/Provisioning\ Profiles \
        ~/Library/MobileDevice/Provisioning\ Profiles \
        -name '*.mobileprovision' -exec \
     sh -c 'security cms -D -i "$1" 2>/dev/null | grep -A1 "<key>Name</key>" | grep "<string>"' _ {} \;
   ```

   If no profile exists for the bundle ID yet, create one by opening any Xcode
   project with automatic signing enabled for the same bundle ID and team, then
   build to a connected device. Xcode will generate the profile automatically.

4. **Regenerate the Xcode project and deploy:**

   ```bash
   bazel run //:xcodeproj
   open imujoco.xcodeproj
   ```

   Select your device in Xcode and hit Run (Cmd+R). The device must have
   **Developer Mode** enabled (Settings → Privacy & Security → Developer Mode).

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
Required for device deployment only. See [Device Deployment](#device-deployment) above.

**No provisioning profile found:**
The `DEV_PROFILE_NAME` in `team_config.bzl` must match an installed profile exactly.
List your profiles and update the name — see [Device Deployment](#device-deployment).

**FlatBuffers / Protobuf codegen errors:**
Ensure schemas build first:
```bash
bazel build //schema:core_schemas
```

**Bazel version mismatch:**
This project requires Bazel 9. Use [Bazelisk](https://github.com/bazelbuild/bazelisk) for automatic version management.

## Work in Progress

- **Network time sync** — Clock synchronization between iMuJoCo on-device and the driver client across Wi-Fi, for accurate latency measurement and coordinated replay

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

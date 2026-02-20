# iMuJoCo

Real-time MuJoCo simulation on Apple devices for your robotics projects

## Overview

iMuJoCo brings the [MuJoCo](https://github.com/google-deepmind/mujoco) physics simulation engine to Apple platforms with native Swift UI and Metal rendering.

- **Swift + Metal** for UI and rendering (no OpenGL)
- **C/C++** for core simulation runtime
- **Swift-C++ interop** for direct calls without bridging overhead

## Project Structure

```
iMuJoCo/
├── imujoco/
│   ├── app/                           # SwiftUI Application
│   │   ├── app/
│   │   │   ├── app.swift              # App entry point
│   │   │   ├── content_view.swift     # Main UI view
│   │   │   └── Assets.xcassets/       # App icons, colors
│   │   └── BUILD.bazel                # iOS/macOS/tvOS app targets
│   │
│   ├── core/                          # Core Runtime Library
│   │   ├── core/
│   │   │   ├── core.swift             # Swift interface to C++ runtime
│   │   │   ├── mjc_physics_runtime.mm # C++ physics runtime
│   │   │   └── module.modulemap       # Swift-C++ interop module
│   │   └── BUILD.bazel                # objc_library + swift_library
│   │
│   └── render/                        # Metal Rendering Library
│       ├── render/
│       │   ├── mjc_metal_render.swift  # Metal render engine
│       │   └── mujoco_shaders.metal   # Metal shaders
│       └── BUILD.bazel                # Metal compilation + swift_library
│
├── driver/                            # C++ Driver Library
│   ├── cc/                            # C++ source and tests
│   ├── python/                        # Python bindings
│   └── BUILD.bazel                    # cc_library + cc_test
│
├── schema/                            # FlatBuffers Schemas
│   ├── control.fbs                    # Control packet schema
│   ├── state.fbs                      # State packet schema
│   └── BUILD.bazel                    # FlatBuffers codegen
│
├── third_party/                       # External Dependencies
│   ├── mujoco.BUILD                   # cc_library for MuJoCo
│   ├── *.BUILD                        # BUILD files for MuJoCo C deps
│   └── patches/                       # Patches for Bazel 9 compatibility
│
├── MODULE.bazel                       # Bazel module config (bzlmod)
├── BUILD.bazel                        # Root: xcodeproj generation
├── extensions.bzl                     # Module extension for C deps
└── .bazelrc                           # Build flags + platform configs
```

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        app (SwiftUI)                        │
│                   UI, Views, User Interaction               │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                     core (Static Library)                   │
│         Swift ←→ C++ Interop, Metal Rendering               │
│         Real-time Simulation Runtime, Network IO            │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│                   MuJoCo (Static Library)                   │
│                     Physics Simulation                      │
└─────────────────────────────────────────────────────────────┘
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
bazel build //imujoco/app:app
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
bazel build //imujoco/app:app --config=ios_device      # iOS device
bazel build //imujoco/app:app --config=ios_sim          # iOS simulator
bazel build //imujoco/app:app_macos --config=macos      # macOS
bazel build //imujoco/app:app_tvos --config=tvos_device # tvOS device
```

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

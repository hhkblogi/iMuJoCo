# iMuJoCo

Real-time MuJoCo streaming on Apple devices for your robotics continuous learning projects

## Overview

iMuJoCo brings the [MuJoCo](https://github.com/google-deepmind/mujoco) physics simulation engine to Apple platforms with native Swift UI and Metal rendering.

- **Swift + Metal** for UI and rendering (no OpenGL)
- **C/C++** for core simulation runtime
- **Swift-C++ interop** for direct calls without bridging overhead

## Project Structure

```
iMuJoCo/
├── app/                            # SwiftUI Application
│   ├── app/
│   │   ├── app.swift               # App entry point
│   │   ├── content_view.swift      # Main UI view
│   │   └── Assets.xcassets/        # App icons, colors
│   ├── app_tests/                  # Unit tests
│   └── app_ui_tests/               # UI tests
│
├── core/                           # Core Runtime Library (Static)
│   ├── core/
│   │   └── core.swift              # Swift interface to C++ runtime
│   └── core_tests/                 # Unit tests
│   # Future:
│   # ├── include/                  # Public C/C++ headers
│   # ├── src/                      # C++23 implementation
│   # │   ├── runtime/              # MuJoCo simulation runtime
│   # │   ├── network/              # Network IO for remote sim
│   # │   └── metal/                # Metal rendering helpers
│   # └── swift/                    # Swift wrappers
│
├── third_party/                    # External dependencies (submodules)
│   ├── mujoco/                     # MuJoCo physics engine (v3.4.0)
│   └── .../                        # Future: ROS, etc.
│
├── build/                          # Build output (gitignored)
│   ├── mujoco/
│   │   ├── ios/
│   │   ├── tvos/
│   │   └── macos/
│   └── .../
│
├── cmake/                          # CMake configurations
│   ├── mujoco/
│   │   └── CMakeLists.txt          # MuJoCo build config
│   └── toolchains/                 # Shared toolchain files
│       ├── ios.toolchain.cmake
│       ├── tvos.toolchain.cmake
│       └── macos.toolchain.cmake
│
├── scripts/                        # Build automation
│   ├── build_mujoco.sh             # Build MuJoCo for all platforms
│   └── build_all.sh                # Build all dependencies
│
└── imujoco.xcworkspace/            # Xcode workspace (all projects)
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
| iOS      | 18.0        | arm64        |
| iPadOS   | 18.0        | arm64        |
| tvOS     | 18.0        | arm64        |
| macOS    | 15.0        | arm64        |

## Getting Started

### Prerequisites

- Xcode 16.0+
- CMake 3.24+
- macOS 15.0+ (for development)

### Clone

```bash
git clone --recurse-submodules https://github.com/hhkblogi/iMuJoCo.git
cd iMuJoCo
```

### Setup Git Hooks

Enable the pre-commit hook to prevent accidental commits of Apple Developer Team IDs:

```bash
git config core.hooksPath .githooks
```

### Build

**Option A: Via Xcode (recommended)**

Open `imujoco.xcworkspace` and build (Cmd+B).
MuJoCo builds automatically via the Aggregate target dependency.

```bash
open imujoco.xcworkspace
```

**Option B: Manual (CI/CD or command line)**

```bash
# Build for specific platform
./scripts/build_mujoco.sh ios        # Build for iOS
./scripts/build_mujoco.sh tvos       # Build for tvOS
./scripts/build_mujoco.sh macos      # Build for macOS

# Build all platforms
./scripts/build_mujoco.sh all        # Build all platforms
./scripts/build_mujoco.sh force-all  # Clean and rebuild all

# Clean build directory
./scripts/build_mujoco.sh clean
```

## License

Apache License 2.0 - see [LICENSE](LICENSE) for details.

### Third-Party Licenses

- [MuJoCo](https://github.com/google-deepmind/mujoco) - Apache License 2.0

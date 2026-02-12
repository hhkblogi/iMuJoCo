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
│   └── build_mujoco_xcframework.sh # Build MuJoCo XCFramework for all platforms
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
| iOS      | 26.0        | arm64        |
| iPadOS   | 26.0        | arm64        |
| tvOS     | 26.0        | arm64        |
| macOS    | 26.0        | arm64        |

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

**Step 1: Build MuJoCo XCFramework**

```bash
# Build MuJoCo dynamic XCFramework (includes all platforms: macOS, iOS, tvOS)
./scripts/build_mujoco_xcframework.sh

# Or build static version (optional)
./scripts/build_mujoco_xcframework.sh --static
```

This creates `build/frameworks/mujoco.xcframework` - a dynamic framework with all dependencies bundled inside.

**Step 2: Open and Build in Xcode**

```bash
open imujoco.xcworkspace
```

Build the project (Cmd+B). The `core` framework links against the MuJoCo XCFramework via `MuJoCoFramework.xcconfig`.

**Note:** For apps using the `core` framework, add `mujoco.xcframework` to "Frameworks, Libraries, and Embedded Content" with **Embed & Sign**.

## TODO

- [ ] Migrate to Bazel for build and package system

## Development Notes

### Future Refinements

- **Swift wrapper necessity**: `mjc_runtime.swift` wraps the C interface for Swift-idiomatic API (properties, throws, deinit). However, Swift can call C directly via the `MJCPhysicsRuntime` module. Consider whether the wrapper adds enough value or if direct C calls would be leaner.

- **Custom logging subsystem**: Currently using `OS_LOG_DEFAULT` for simplicity. For production, consider using `os_log_create()` with custom subsystem/category (e.g., `"com.mujoco.core"`, `"UDPServer"`) to enable filtering in Console.app and per-component log retention policies.

## License

Apache License 2.0 - see [LICENSE](LICENSE) for details.

### Third-Party Licenses

- [MuJoCo](https://github.com/google-deepmind/mujoco) - Apache License 2.0

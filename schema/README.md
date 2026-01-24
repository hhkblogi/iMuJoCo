# iMuJoCo Schema Definitions

FlatBuffers schema files for iMuJoCo data streaming protocol.

## Structure

```
schema/
├── control.fbs    # driver → iMuJoCo (actuator controls)
└── state.fbs      # iMuJoCo → driver (simulation state)

build/generated/flatbuffers/   # Generated C++ headers (not in repo)
└── imujoco/
    └── schema/                # Mirrors namespace imujoco.schema
        ├── control_generated.h
        └── state_generated.h
```

## Xcode Integration

The `flatbuffers-codegen` aggregate target generates headers before `core` builds.
This follows the same pattern as `mujoco-framework`.

**Build order:** `flatbuffers-codegen` → `mujoco-framework` → `core`

Headers are output to `build/generated/flatbuffers/imujoco/schema/` and included via
`HEADER_SEARCH_PATHS` in `third_party/FlatBuffers.xcconfig`.

### First-time setup

Build `flatc` from the submodule (one-time, from terminal):
```bash
./scripts/generate_flatbuffers.sh
```

After this, Xcode builds will auto-regenerate headers when `.fbs` files change.

### Usage in C++

```cpp
#include "imujoco/schema/state_generated.h"
#include "imujoco/schema/control_generated.h"

using namespace imujoco::schema;
```

## Manual Generation (CLI)

```bash
# Generate to default location (build/generated/flatbuffers)
./scripts/generate_flatbuffers.sh

# Generate to custom location
./scripts/generate_flatbuffers.sh /path/to/output

# Generate Python bindings
./build/flatc/flatc --python -o /tmp/python schema/*.fbs
```

## Protocol Overview

```
┌────────────┐    ControlPacket     ┌────────────┐
│   Driver   │  ─────────────────►  │  iMuJoCo   │
│            │  ◄─────────────────  │            │
└────────────┘    StatePacket       └────────────┘
```

- **ControlPacket** (file_identifier: "CTPK"): Send actuator control values
- **StatePacket** (file_identifier: "STPK"): Receive simulation state (qpos, qvel, sensors)

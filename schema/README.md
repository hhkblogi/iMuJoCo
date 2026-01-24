# iMuJoCo Schema Definitions

FlatBuffers schema files for iMuJoCo data streaming protocol.

## Structure

```
schema/
└── mujoco/
    ├── control.fbs    # driver → iMuJoCo (actuator controls)
    └── state.fbs      # iMuJoCo → driver (simulation state)
```

## Generating Code

Install FlatBuffers compiler:
```bash
brew install flatbuffers
```

Generate C++ headers:
```bash
flatc --cpp -o generated/cpp schema/mujoco/*.fbs
```

Generate Python:
```bash
flatc --python -o generated/python schema/mujoco/*.fbs
```

## Protocol Overview

```
┌────────────┐    ControlPacket     ┌────────────┐
│   Driver   │  ─────────────────►  │  iMuJoCo   │
│            │  ◄─────────────────  │            │
└────────────┘    StatePacket       └────────────┘
```

- **ControlPacket**: Send actuator control values
- **StatePacket**: Receive simulation state (qpos, qvel, sensors)

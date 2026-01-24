# iMuJoCo Schema Definitions

FlatBuffers schema files for iMuJoCo data streaming protocol.

## Structure

```
schema/
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
flatc --cpp -o generated/cpp schema/*.fbs
```

Generate Python:
```bash
flatc --python -o generated/python schema/*.fbs
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

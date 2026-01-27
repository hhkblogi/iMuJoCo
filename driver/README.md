# imujoco-driver

Python driver for iMuJoCo - UDP client for MuJoCo simulation on iOS.

## Installation

```bash
pip install driver/
```

## Quick Start

```python
from imujoco_driver import Driver, DriverConfig
import threading
import time

# Configure driver
config = DriverConfig()
config.host = "192.168.1.100"  # Your iPhone's IP
config.port = 8888

# Create and connect
driver = Driver(config)
driver.connect()

# Thread-safe state storage
latest_state = None
state_lock = threading.Lock()

def on_state(state):
    global latest_state
    with state_lock:
        latest_state = state

# Subscribe to state updates
sub_id = driver.subscribe(on_state)

# Send initial control to kickstart communication
driver.send_control([])

# Control loop
for _ in range(100):
    with state_lock:
        if latest_state:
            print(f"time={latest_state.time:.3f}")
    driver.send_control([0.0] * 10)  # Example control
    time.sleep(0.02)

# Cleanup
driver.unsubscribe(sub_id)
driver.disconnect()
```

## API

### DriverConfig
- `host`: Target simulation host (default: "127.0.0.1")
- `port`: Target simulation UDP port (default: 8888)
- `timeout_ms`: Receive timeout in milliseconds (default: 100)

### Driver
- `connect()` / `disconnect()`: Connection management
- `send_control(ctrl)`: Send control values (list of floats)
- `subscribe(callback)`: Subscribe to state updates
- `unsubscribe(id)`: Unsubscribe from updates
- `get_stats()`: Get driver statistics

### SimulationState
- `time`: Simulation time
- `qpos`: Joint positions
- `qvel`: Joint velocities
- `ctrl`: Current control values
- `sensordata`: Sensor readings

## License

Apache-2.0

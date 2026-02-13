# imujoco_driver - Python bindings for iMuJoCo driver
#
# Thread-safe UDP client with subscriber model for state updates.
# Send and receive happen independently at different rates.
#
# Threading Model:
#   - TX (send_control): Runs on caller's thread
#   - RX (callbacks): Runs on dedicated RX thread
#   - All public methods are thread-safe
#
# Important: State callbacks run on the RX thread. Keep callbacks lightweight
# or dispatch work to your own thread to avoid blocking state reception.

from ._imujoco_driver import (
    Driver,
    DriverConfig,
    SimulationState,
    ControlCommand,
    DriverStats,
    __version__,
)

__all__ = [
    "Driver",
    "DriverConfig",
    "SimulationState",
    "ControlCommand",
    "DriverStats",
]

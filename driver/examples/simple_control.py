#!/usr/bin/env python3
"""
simple_control.py - Example of using imujoco_driver

Demonstrates the async callback pattern:
- Subscribe to state updates (runs on RX thread)
- Send control commands (runs on main thread)
- Send and receive happen at different rates
"""

import argparse
import math
import threading
import time

from imujoco_driver import Driver, DriverConfig


def main():
    parser = argparse.ArgumentParser(description="iMuJoCo simple control example")
    parser.add_argument("--host", default="127.0.0.1", help="iMuJoCo host")
    parser.add_argument("--port", type=int, default=9001, help="iMuJoCo port")
    parser.add_argument("--duration", type=float, default=5.0, help="Run duration in seconds")
    parser.add_argument("--rate", type=float, default=50.0, help="Control rate in Hz")
    args = parser.parse_args()

    # Validate rate to prevent division by zero
    if args.rate <= 0:
        print("Error: --rate must be positive")
        return 1

    # Configure driver
    config = DriverConfig()
    config.host = args.host
    config.port = args.port
    config.timeout_ms = 100

    # Create driver
    driver = Driver(config)

    # Thread-safe state storage
    latest_state = None
    state_lock = threading.Lock()
    state_count = 0

    def on_state(state):
        """Called on RX thread when state is received."""
        nonlocal latest_state, state_count
        with state_lock:
            latest_state = state
            state_count += 1
            if state_count % 50 == 0:  # Print every ~1 second at 50Hz
                print(f"  [RX] time={state.time:.3f}s qpos[{len(state.qpos)}] qvel[{len(state.qvel)}]")

    def on_error(code, message):
        """Called on RX thread when error occurs."""
        print(f"  [ERROR] code={code} msg={message}")

    # Connect
    print(f"Connecting to {args.host}:{args.port}...")
    if not driver.connect():
        print("Failed to connect!")
        return 1

    print("Connected!")

    # Subscribe to state updates
    sub_id = driver.subscribe(on_state)
    driver.on_error(on_error)

    # Send an initial empty control to kickstart communication
    # (iMuJoCo sends state in response to control)
    driver.send_control([])

    # Control loop
    print(f"Running for {args.duration}s at {args.rate}Hz...")
    start_time = time.time()
    control_count = 0
    dt = 1.0 / args.rate

    try:
        while time.time() - start_time < args.duration:
            # Create control command (example: sinusoidal control)
            t = time.time() - start_time

            # Get current state to determine control size
            with state_lock:
                if latest_state is not None and len(latest_state.qpos) > 0:
                    # Create sinusoidal control values
                    num_ctrl = len(latest_state.qpos)
                    ctrl = [0.1 * math.sin(2 * math.pi * t + i) for i in range(num_ctrl)]
                else:
                    ctrl = []

            # Send control
            driver.send_control(ctrl)
            control_count += 1

            # Sleep to maintain rate
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nInterrupted by user")

    # Cleanup
    driver.unsubscribe(sub_id)
    driver.disconnect()

    # Print stats
    stats = driver.get_stats()
    elapsed = time.time() - start_time

    # Avoid division by zero for very short runs
    if elapsed > 0:
        tx_rate = stats.packets_sent / elapsed
        rx_rate = stats.packets_received / elapsed
        ctrl_rate = control_count / elapsed
    else:
        tx_rate = rx_rate = ctrl_rate = 0.0

    print(f"\nStats:")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Controls sent: {control_count} ({ctrl_rate:.1f}/s)")
    print(f"  TX packets: {stats.packets_sent} ({tx_rate:.1f}/s)")
    print(f"  RX packets: {stats.packets_received} ({rx_rate:.1f}/s)")
    print(f"  TX errors: {stats.send_errors}")
    print(f"  RX errors: {stats.receive_errors}")

    return 0


if __name__ == "__main__":
    exit(main())

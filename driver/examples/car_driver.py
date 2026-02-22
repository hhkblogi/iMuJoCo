#!/usr/bin/env python3
"""
car_driver.py - MuJoCo car model driver example

The car has 2 actuators:
  ctrl[0] = forward  (drive both wheels, range -1 to 1)
  ctrl[1] = turn     (differential steering, range -1 to 1)

Modes:
  circle   - Drive in a circle (default)
  figure8  - Drive in a figure-8 pattern
  drift    - Accelerate then spin

Usage:
  bazel run //driver:car_driver -- --host <device_ip> --mode circle
"""

import argparse
import math
import threading
import time

from imujoco_driver import Driver, DriverConfig

# =============================================================================
# Car model constants
# =============================================================================

NUM_CTRL = 2
QPOS_BASE = 7  # freejoint: 3 pos + 4 quat


def quat_to_yaw(w, x, y, z):
    """Extract yaw angle (degrees) from quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def format_state(state):
    """Format car state as a single-line summary."""
    qpos = state.qpos
    if len(qpos) >= QPOS_BASE:
        x, y, z = qpos[0], qpos[1], qpos[2]
        yaw = quat_to_yaw(qpos[3], qpos[4], qpos[5], qpos[6])
        ctrl = state.ctrl
        fwd = ctrl[0] if len(ctrl) > 0 else 0
        trn = ctrl[1] if len(ctrl) > 1 else 0
        return (f"t={state.time:6.2f}s  pos=({x:+.2f},{y:+.2f},{z:+.3f})  "
                f"yaw={yaw:+6.1f}°  ctrl=[{fwd:+.2f},{trn:+.2f}]  "
                f"KE={state.energy_kinetic:.4f}")
    return f"t={state.time:.2f}s (waiting for state...)"


def ctrl_circle(t):
    """Steady forward + constant turn."""
    return [0.6, 0.3]


def ctrl_figure8(t):
    """Forward drive with sinusoidal steering."""
    return [0.5, 0.4 * math.sin(2 * math.pi * t / 6.0)]


def ctrl_drift(t):
    """Accelerate, then hard spin."""
    if t < 2.0:
        return [1.0, 0.0]  # full throttle straight
    else:
        return [0.8, 1.0]  # throttle + hard turn


MODES = {
    "circle": ctrl_circle,
    "figure8": ctrl_figure8,
    "drift": ctrl_drift,
}


def main():
    parser = argparse.ArgumentParser(description="iMuJoCo car driver")
    parser.add_argument("--host", default="127.0.0.1", help="iMuJoCo host")
    parser.add_argument("--port", type=int, default=9001, help="iMuJoCo port")
    parser.add_argument("--mode", default="circle", choices=list(MODES.keys()),
                        help="Driving mode")
    parser.add_argument("--duration", type=float, default=10.0, help="Run duration (seconds)")
    parser.add_argument("--rate", type=float, default=50.0, help="Control rate (Hz)")
    args = parser.parse_args()

    if args.rate <= 0:
        print("Error: --rate must be positive")
        return 1

    ctrl_fn = MODES[args.mode]

    print(f"Car Driver  mode={args.mode}  duration={args.duration}s  rate={args.rate}Hz")

    # Configure and connect
    config = DriverConfig()
    config.host = args.host
    config.port = args.port
    config.timeout_ms = 100

    driver = Driver(config)

    latest_state = None
    state_lock = threading.Lock()
    state_count = 0
    print_interval = max(1, int(args.rate))  # ~1 Hz print

    def on_state(state):
        nonlocal latest_state, state_count
        with state_lock:
            latest_state = state
            state_count += 1
            if state_count % print_interval == 0:
                print(f"  {format_state(state)}")

    def on_error(code, message):
        print(f"  [ERROR] code={code} msg={message}")

    print(f"Connecting to {args.host}:{args.port}...")
    if not driver.connect():
        print("Failed to connect!")
        return 1
    print("Connected!")

    sub_id = driver.subscribe(on_state)
    driver.on_error(on_error)

    # Kickstart communication
    driver.send_control([])

    # Control loop
    print(f"Driving ({args.mode})...")
    start_time = time.time()
    control_count = 0
    dt = 1.0 / args.rate

    try:
        while time.time() - start_time < args.duration:
            t = time.time() - start_time
            ctrl = ctrl_fn(t)
            driver.send_control(ctrl)
            control_count += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nInterrupted by user")

    # Stop the car
    driver.send_control([0.0, 0.0])

    # Cleanup
    driver.unsubscribe(sub_id)
    driver.disconnect()

    # Stats
    stats = driver.get_stats()
    elapsed = time.time() - start_time
    if elapsed > 0:
        tx_rate = stats.packets_sent / elapsed
        rx_rate = stats.packets_received / elapsed
    else:
        tx_rate = rx_rate = 0.0

    print(f"\nStats:")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Controls sent: {control_count} ({control_count / max(elapsed, 1e-9):.1f}/s)")
    print(f"  TX: {stats.packets_sent} ({tx_rate:.1f}/s)  RX: {stats.packets_received} ({rx_rate:.1f}/s)")
    print(f"  Errors: TX={stats.send_errors} RX={stats.receive_errors}")

    return 0


if __name__ == "__main__":
    exit(main())

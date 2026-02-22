#!/usr/bin/env python3
"""
unitree_g1_driver.py - Unitree G1 robot driver example

Control modes:
  stand  - Hold standing pose (keyframe ctrl values)
  zero   - Drive all joints to 0 rad
  free   - Send zero ctrl (identical to zero; true actuator-free requires app support)

Usage:
  bazel run //driver:unitree_g1_driver -- --host <device_ip> --mode stand
"""

import argparse
import math
import threading
import time

from imujoco_driver import Driver, DriverConfig

# =============================================================================
# Unitree G1 model constants
# =============================================================================

NUM_CTRL = 29
QPOS_BASE = 7  # freejoint: 3 pos + 4 quat
QVEL_BASE = 6  # freejoint: 3 vel + 3 angvel

# Standing keyframe ctrl (from Unitree G1 MJCF)
STAND_CTRL = [
    # Left leg (0-5): hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
    0, 0, 0, 0, 0, 0,
    # Right leg (6-11)
    0, 0, 0, 0, 0, 0,
    # Waist (12-14): yaw, roll, pitch
    0, 0, 0,
    # Left arm (15-21): shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw
    0.2, 0.2, 0, 1.28, 0, 0, 0,
    # Right arm (22-28)
    0.2, -0.2, 0, 1.28, 0, 0, 0,
]

# Joint names by group (matches ctrl index order)
JOINT_GROUPS = [
    ("Left leg", [
        "hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll",
    ]),
    ("Right leg", [
        "hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll",
    ]),
    ("Waist", ["yaw", "roll", "pitch"]),
    ("Left arm", [
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
        "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw",
    ]),
    ("Right arm", [
        "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
        "elbow", "wrist_roll", "wrist_pitch", "wrist_yaw",
    ]),
]

# Sensor layout: 2 gyros (3 each) + 2 accelerometers (3 each) = 12 values
SENSORS = [
    ("Torso gyro", 0, 3),
    ("Pelvis gyro", 3, 6),
    ("Torso accel", 6, 9),
    ("Pelvis accel", 9, 12),
]


def quat_to_euler(w, x, y, z):
    """Convert quaternion to euler angles (roll, pitch, yaw) in degrees."""
    # Roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def format_state(state):
    """Format simulation state as human-readable string."""
    lines = []
    qpos = state.qpos
    qvel = state.qvel
    sensordata = state.sensordata

    lines.append(f"{'=' * 60}")
    lines.append(f"  t = {state.time:.3f}s    "
                 f"KE = {state.energy_kinetic:.2f}  PE = {state.energy_potential:.2f}")

    # Base pose
    if len(qpos) >= QPOS_BASE:
        x, y, z = qpos[0], qpos[1], qpos[2]
        qw, qx, qy, qz = qpos[3], qpos[4], qpos[5], qpos[6]
        roll, pitch, yaw = quat_to_euler(qw, qx, qy, qz)
        lines.append(f"  Base pos: ({x:+.3f}, {y:+.3f}, {z:+.3f})")
        lines.append(f"  Base ori: roll={roll:+.1f}  pitch={pitch:+.1f}  yaw={yaw:+.1f} deg")

    # Joint angles by group
    joint_qpos = qpos[QPOS_BASE:]  # skip freejoint
    idx = 0
    for group_name, joint_names in JOINT_GROUPS:
        n = len(joint_names)
        if idx + n > len(joint_qpos):
            break
        angles = [math.degrees(joint_qpos[idx + j]) for j in range(n)]
        parts = [f"{name}={ang:+6.1f}" for name, ang in zip(joint_names, angles)]
        lines.append(f"  {group_name:>10s}: {', '.join(parts)}")
        idx += n

    # Sensors
    if len(sensordata) >= 12:
        for name, start, end in SENSORS:
            vals = sensordata[start:end]
            lines.append(f"  {name:>14s}: ({vals[0]:+7.3f}, {vals[1]:+7.3f}, {vals[2]:+7.3f})")

    lines.append(f"{'=' * 60}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="iMuJoCo Unitree G1 robot driver")
    parser.add_argument("--host", default="127.0.0.1", help="iMuJoCo host")
    parser.add_argument("--port", type=int, default=9001, help="iMuJoCo port")
    parser.add_argument("--mode", default="stand", choices=["stand", "zero", "free"],
                        help="Control mode: stand (hold pose), zero (all joints to 0), free (no ctrl)")
    parser.add_argument("--duration", type=float, default=10.0, help="Run duration in seconds")
    parser.add_argument("--rate", type=float, default=50.0, help="Control rate in Hz")
    args = parser.parse_args()

    if args.rate <= 0:
        print("Error: --rate must be positive")
        return 1

    # Build ctrl for the selected mode
    # Note: the app ignores empty ctrl arrays (keeps last values), so all modes
    # must send NUM_CTRL values. True actuator-free simulation would require
    # app-side support to zero out actuator gains.
    if args.mode == "stand":
        ctrl = list(STAND_CTRL)
    else:  # zero or free — both send zeros (drives joints to 0 rad)
        ctrl = [0.0] * NUM_CTRL

    print(f"Unitree G1 Driver  mode={args.mode}  ctrl_len={len(ctrl)}  "
          f"duration={args.duration}s  rate={args.rate}Hz")

    # Configure and connect
    config = DriverConfig()
    config.host = args.host
    config.port = args.port
    config.timeout_ms = 100

    driver = Driver(config)

    # Thread-safe state storage
    latest_state = None
    state_lock = threading.Lock()
    state_count = 0
    print_interval = max(1, int(args.rate))  # print ~every 1 second

    def on_state(state):
        nonlocal latest_state, state_count
        with state_lock:
            latest_state = state
            state_count += 1
            if state_count % print_interval == 0:
                print(format_state(state))

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
    print(f"Sending {args.mode} ctrl...")
    start_time = time.time()
    control_count = 0
    dt = 1.0 / args.rate

    try:
        while time.time() - start_time < args.duration:
            driver.send_control(ctrl)
            control_count += 1
            time.sleep(dt)
    except KeyboardInterrupt:
        print("\nInterrupted by user")

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

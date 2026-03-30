#!/usr/bin/env python3
"""Verify force data fields in StatePacket from a running iMuJoCo instance.

Usage:
    bazel run //driver:verify_force_data -- --host <iPad-IP>
"""

import argparse
import sys
import time

import imujoco_driver


def main():
    parser = argparse.ArgumentParser(description="Verify force data in StatePacket")
    parser.add_argument("--host", required=True, help="Device IP address")
    parser.add_argument("--port", type=int, default=9000, help="UDP port (default: 9000)")
    args = parser.parse_args()

    config = imujoco_driver.DriverConfig()
    config.host = args.host
    config.port = args.port

    driver = imujoco_driver.Driver(config)
    if not driver.connect():
        print(f"ERROR: Failed to connect to {args.host}:{args.port}", file=sys.stderr)
        return 1

    results = []

    def on_state(state):
        try:
            # Just access sequence to test basic field access
            _ = state.sequence
            results.append(state)
        except Exception as e:
            print(f"CALLBACK ERROR: {e}", flush=True)

    driver.subscribe(on_state)

    print(f"Connected to {args.host}:{args.port}, waiting for state packets...")
    sys.stdout.flush()

    # Keep sending empty control to trigger state responses
    for i in range(50):
        driver.send_control([])
        time.sleep(0.1)
        if len(results) >= 5:
            print(f"Got {len(results)} packets after {i+1} sends")
            sys.stdout.flush()
            break

    print(f"Disconnecting... (collected {len(results)} packets)")
    sys.stdout.flush()
    driver.disconnect()
    print("Disconnected.")
    sys.stdout.flush()

    if not results:
        print("ERROR: No state packets received")
        return 1

    print(f"\nReceived {len(results)} state packets\n")

    # Check the last state
    s = results[-1]
    print(f"=== StatePacket (seq={s.sequence}, time={s.time:.4f}) ===\n")

    # Old fields
    print(f"qpos[{len(s.qpos)}]: {s.qpos[:5]}{'...' if len(s.qpos) > 5 else ''}")
    print(f"qvel[{len(s.qvel)}]: {s.qvel[:5]}{'...' if len(s.qvel) > 5 else ''}")
    print(f"ctrl[{len(s.ctrl)}]")
    print(f"sensordata[{len(s.sensordata)}]")
    print()

    # Force fields
    errors = []

    print("=== Force Data ===\n")

    print(f"qfrc_actuator[{len(s.qfrc_actuator)}]: {s.qfrc_actuator[:5]}{'...' if len(s.qfrc_actuator) > 5 else ''}")
    if len(s.qfrc_actuator) == 0:
        errors.append("qfrc_actuator is empty (expected length = nv)")

    print(f"qfrc_bias[{len(s.qfrc_bias)}]: {s.qfrc_bias[:5]}{'...' if len(s.qfrc_bias) > 5 else ''}")
    if len(s.qfrc_bias) == 0:
        errors.append("qfrc_bias is empty (expected length = nv)")
    elif all(v == 0.0 for v in s.qfrc_bias):
        errors.append("qfrc_bias is all zeros (expected gravity component)")

    print(f"cfrc_int[{len(s.cfrc_int)}]: {s.cfrc_int[:6]}{'...' if len(s.cfrc_int) > 6 else ''}")
    if len(s.cfrc_int) == 0:
        errors.append("cfrc_int is empty (expected length = nbody * 6)")

    print(f"cfrc_ext[{len(s.cfrc_ext)}]: {s.cfrc_ext[:6]}{'...' if len(s.cfrc_ext) > 6 else ''}")
    if len(s.cfrc_ext) == 0:
        errors.append("cfrc_ext is empty (expected length = nbody * 6)")

    print()
    print("=== Contact Data ===\n")

    print(f"ncon: {s.ncon}")
    print(f"contact_pos[{len(s.contact_pos)}]: {s.contact_pos[:6]}{'...' if len(s.contact_pos) > 6 else ''}")
    print(f"contact_force[{len(s.contact_force)}]: {s.contact_force[:5]}{'...' if len(s.contact_force) > 5 else ''}")
    print(f"contact_body1[{len(s.contact_body1)}]: {s.contact_body1[:5]}{'...' if len(s.contact_body1) > 5 else ''}")
    print(f"contact_body2[{len(s.contact_body2)}]: {s.contact_body2[:5]}{'...' if len(s.contact_body2) > 5 else ''}")

    if s.ncon > 0:
        if len(s.contact_pos) != s.ncon * 3:
            errors.append(f"contact_pos length {len(s.contact_pos)} != ncon*3={s.ncon*3}")
        if len(s.contact_force) != s.ncon:
            errors.append(f"contact_force length {len(s.contact_force)} != ncon={s.ncon}")
        if len(s.contact_body1) != s.ncon:
            errors.append(f"contact_body1 length {len(s.contact_body1)} != ncon={s.ncon}")
        if len(s.contact_body2) != s.ncon:
            errors.append(f"contact_body2 length {len(s.contact_body2)} != ncon={s.ncon}")
        if any(f < 0 for f in s.contact_force):
            errors.append("contact_force has negative values")

    print()
    if errors:
        print("=== ERRORS ===")
        for e in errors:
            print(f"  FAIL: {e}")
        return 1
    else:
        print("=== ALL CHECKS PASSED ===")
        return 0


if __name__ == "__main__":
    sys.exit(main())

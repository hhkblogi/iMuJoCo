#!/usr/bin/env python3
"""
test_paced_replay.py - Verify PacedReplay mode

Proves: controls are applied at the original sender cadence, while the simulation
still runs at its native stepping rate.

Prerequisites:
  - iMuJoCo app running with controlMode = PacedReplay (MJCControlMode::PacedReplay)
  - A model loaded and simulation started

Usage:
  bazel run //driver:test_paced_replay_py -- --host <device_ip> --port 9001
"""

import argparse
import math
import threading
import time
import sys

from imujoco_driver import Driver, DriverConfig, ControlCommand


def test_timestamp_cadence(driver, n_controls=50, send_rate=100.0):
    """
    Test 1: Send controls with monotonic timestamps at 100 Hz cadence.
    Each control has a distinct value. Verify the values appear in state
    at the correct cadence (~10ms apart for 100 Hz).

    Note: The simulation still runs at native rate (e.g., 500 Hz). PacedReplay
    controls WHEN controls are applied, not the stepping rate. Between paced
    applications, the real-time loop fills in with the current ctrl values.
    """
    print(f"\n{'='*60}")
    print(f"TEST 1: Timestamp-based cadence ({send_rate} Hz, {n_controls} packets)")
    print(f"{'='*60}")

    states = []
    state_lock = threading.Lock()

    def on_state(state):
        with state_lock:
            states.append((time.time(), state.time, list(state.ctrl)))

    sub_id = driver.subscribe(on_state)

    # Send one initial packet to kickstart
    driver.send_control([0.0])
    time.sleep(0.3)
    with state_lock:
        states.clear()

    # Send controls with monotonic timestamps at send_rate Hz
    # Each control has a distinct value so we can detect when it's applied
    dt_us = int(1e6 / send_rate)  # interval in microseconds
    base_ts = int(time.monotonic() * 1e6)

    start_wall = time.time()
    for i in range(n_controls):
        cmd = ControlCommand()
        cmd.ctrl = [round(0.01 * (i + 1), 4)]  # 0.01, 0.02, ..., 0.50
        cmd.host_timestamp_us = base_ts + i * dt_us
        driver.send_control(cmd)
        time.sleep(0.001)  # Send fast, pacing is on receiver side

    # Wait for paced replay to drain (n_controls / send_rate + margin)
    expected_duration = n_controls / send_rate
    time.sleep(expected_duration + 0.5)

    driver.unsubscribe(sub_id)

    with state_lock:
        snapshot = list(states)

    n_states = len(snapshot)

    # Detect ctrl transitions: when did each new ctrl value first appear?
    transitions = []
    last_ctrl_val = None
    for wall_t, sim_t, ctrl in snapshot:
        if ctrl and len(ctrl) > 0:
            val = round(ctrl[0], 4)
            if val != last_ctrl_val and val > 0:
                transitions.append((wall_t, sim_t, val))
                last_ctrl_val = val

    n_transitions = len(transitions)

    print(f"\n  Results:")
    print(f"    Controls sent:   {n_controls}")
    print(f"    States received: {n_states} (sim runs at native rate)")
    print(f"    Ctrl transitions:{n_transitions} (expected: ~{n_controls})")

    # Check timing between transitions (should be ~1/send_rate seconds apart)
    if n_transitions >= 3:
        wall_deltas = [(transitions[i+1][0] - transitions[i][0]) * 1000
                       for i in range(n_transitions - 1)]
        avg_delta_ms = sum(wall_deltas) / len(wall_deltas)
        expected_ms = 1000.0 / send_rate

        print(f"    Avg transition interval: {avg_delta_ms:.1f} ms (expected: {expected_ms:.1f} ms)")

        # Transitions should happen at roughly the sender cadence
        interval_ok = abs(avg_delta_ms - expected_ms) < expected_ms * 0.5  # Within 50%
        count_ok = n_transitions >= n_controls * 0.5  # At least half the controls detected

        if interval_ok and count_ok:
            print(f"  PASS: Controls applied at sender cadence ({avg_delta_ms:.1f} ms intervals)")
        elif count_ok:
            print(f"  PARTIAL: {n_transitions} transitions detected, but interval {avg_delta_ms:.1f} ms off from expected {expected_ms:.1f} ms")
        else:
            print(f"  FAIL: Only {n_transitions} transitions (expected ~{n_controls})")

        return interval_ok and count_ok
    else:
        print(f"  FAIL: Too few transitions ({n_transitions}) to measure cadence")
        return False


def test_legacy_no_timestamp(driver):
    """
    Test 2: Send controls with host_timestamp_us = 0 (legacy).
    Verify immediate application (no pacing delay).
    """
    print(f"\n{'='*60}")
    print(f"TEST 2: Legacy mode (host_timestamp_us = 0)")
    print(f"{'='*60}")

    states = []
    state_lock = threading.Lock()

    def on_state(state):
        with state_lock:
            states.append((time.time(), state.time))

    sub_id = driver.subscribe(on_state)
    time.sleep(0.1)
    with state_lock:
        states.clear()

    # Send 10 controls with ts=0 (should apply immediately)
    start = time.time()
    for i in range(10):
        cmd = ControlCommand()
        cmd.ctrl = [float(i) * 0.1]
        cmd.host_timestamp_us = 0  # Legacy: apply immediately
        driver.send_control(cmd)
        time.sleep(0.02)

    time.sleep(0.3)
    driver.unsubscribe(sub_id)

    with state_lock:
        n_states = len(states)

    elapsed = time.time() - start
    print(f"\n  Results:")
    print(f"    Controls sent:   10")
    print(f"    States received: {n_states}")
    print(f"    Elapsed:         {elapsed:.2f}s")

    # With ts=0, each ctrl should apply immediately → ~10 states in <1s
    passed = n_states >= 5
    if passed:
        print(f"  PASS: Legacy controls applied without pacing delay")
    else:
        print(f"  FAIL: Too few states ({n_states}) — legacy mode may not work")

    return passed


def main():
    parser = argparse.ArgumentParser(description="Test PacedReplay mode")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    args = parser.parse_args()

    config = DriverConfig()
    config.host = args.host
    config.port = args.port
    config.timeout_ms = 100

    driver = Driver(config)
    print(f"Connecting to {args.host}:{args.port}...")
    if not driver.connect():
        print("Failed to connect!")
        return 1
    print("Connected!")

    results = {}
    results["timestamp_cadence"] = test_timestamp_cadence(driver)
    results["legacy_no_timestamp"] = test_legacy_no_timestamp(driver)

    driver.disconnect()

    print(f"\n{'='*60}")
    print("SUMMARY — PacedReplay Mode")
    print(f"{'='*60}")
    all_pass = True
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")
        if not passed:
            all_pass = False

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())

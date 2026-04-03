#!/usr/bin/env python3
"""
test_control_modes.py - Verify control mode decoupling

Tests that the simulation runs at native physics rate regardless of
driver send rate (Live mode). Also tests extended control fields.

Prerequisites:
  - iMuJoCo app running in Xcode (macOS target)
  - A model loaded (e.g., simple_pendulum via gRPC or UI)

Usage:
  # From repo root, with bazel-built Python module:
  PYTHONPATH=bazel-bin/driver/imujoco_driver python3 driver/examples/test_control_modes.py

  # Or with custom port:
  PYTHONPATH=bazel-bin/driver/imujoco_driver python3 driver/examples/test_control_modes.py --port 9001
"""

import argparse
import math
import threading
import time
import sys

from imujoco_driver import Driver, DriverConfig, ControlCommand


def test_live_mode_rate_decoupling(driver, duration=3.0, send_rate=50.0):
    """
    Test 1: Live mode - sim rate should be independent of driver send rate.

    Send controls at 50 Hz for 3 seconds. The simulation should step at its
    native rate (e.g., 500 Hz for 2ms timestep), NOT at 50 Hz.
    """
    print(f"\n{'='*60}")
    print(f"TEST 1: Live mode rate decoupling")
    print(f"  Sending at {send_rate} Hz for {duration}s")
    print(f"  Expecting sim to run at native rate (>> {send_rate} Hz)")
    print(f"{'='*60}")

    states = []
    state_lock = threading.Lock()

    def on_state(state):
        with state_lock:
            states.append((time.time(), state.time))

    sub_id = driver.subscribe(on_state)

    # Send controls at the specified rate
    dt = 1.0 / send_rate
    start = time.time()
    send_count = 0

    while time.time() - start < duration:
        t = time.time() - start
        ctrl = [0.1 * math.sin(2 * math.pi * 0.5 * t)]
        driver.send_control(ctrl)
        send_count += 1
        time.sleep(dt)

    driver.unsubscribe(sub_id)
    elapsed = time.time() - start

    with state_lock:
        n_states = len(states)

    send_hz = send_count / elapsed
    state_hz = n_states / elapsed if elapsed > 0 else 0

    # Check sim_time advancement
    if n_states >= 2:
        sim_start = states[0][1]
        sim_end = states[-1][1]
        sim_elapsed = sim_end - sim_start
        sim_ratio = sim_elapsed / elapsed if elapsed > 0 else 0
    else:
        sim_elapsed = 0
        sim_ratio = 0

    print(f"\n  Results:")
    print(f"    Duration:        {elapsed:.2f}s")
    print(f"    Controls sent:   {send_count} ({send_hz:.1f} Hz)")
    print(f"    States received: {n_states} ({state_hz:.1f} Hz)")
    print(f"    Sim time elapsed:{sim_elapsed:.3f}s (ratio: {sim_ratio:.2f}x)")

    # The key check: state receive rate should be much higher than send rate
    # because sim steps at native rate and sends state every step
    passed = state_hz > send_rate * 1.5  # Should be at least 1.5x driver rate
    if passed:
        print(f"  PASS: State rate ({state_hz:.0f} Hz) >> driver rate ({send_hz:.0f} Hz)")
    else:
        print(f"  FAIL: State rate ({state_hz:.0f} Hz) not significantly higher than driver rate ({send_hz:.0f} Hz)")
        print(f"        (This could mean simulation is still lock-stepped to driver)")

    return passed


def test_extended_fields(driver):
    """
    Test 2: Extended control fields - send qfrc_applied and verify state response.
    """
    print(f"\n{'='*60}")
    print(f"TEST 2: Extended control fields (qfrc_applied)")
    print(f"{'='*60}")

    states = []
    state_lock = threading.Lock()

    def on_state(state):
        with state_lock:
            states.append(state)

    sub_id = driver.subscribe(on_state)

    # Send a ControlCommand with qfrc_applied set
    cmd = ControlCommand()
    cmd.ctrl = [0.5]  # Also send a ctrl value
    cmd.qfrc_applied = [1.0]  # Direct joint force
    driver.send_control(cmd)

    # Wait for some states
    time.sleep(0.5)

    driver.unsubscribe(sub_id)

    with state_lock:
        n_states = len(states)

    print(f"\n  Results:")
    print(f"    States received: {n_states}")

    if n_states > 0:
        last = states[-1]
        print(f"    Last sim time:   {last.time:.4f}s")
        print(f"    Last ctrl:       {list(last.ctrl)[:3]}")
        print(f"    Last qpos:       {list(last.qpos)[:3]}")
        print(f"  PASS: Extended fields accepted (no crash, states flowing)")
        return True
    else:
        print(f"  FAIL: No states received after sending extended fields")
        return False


def test_ctrl_timeout(driver, timeout_ms=500):
    """
    Test 3: Ctrl timeout - stop sending, verify actuators zero.
    """
    print(f"\n{'='*60}")
    print(f"TEST 3: Control timeout ({timeout_ms}ms)")
    print(f"{'='*60}")

    states = []
    state_lock = threading.Lock()

    def on_state(state):
        with state_lock:
            states.append(state)

    sub_id = driver.subscribe(on_state)

    # Send non-zero ctrl for a bit
    print(f"  Sending ctrl=[1.0] for 0.5s...")
    start = time.time()
    while time.time() - start < 0.5:
        driver.send_control([1.0])
        time.sleep(0.02)

    # Record ctrl value before timeout
    with state_lock:
        if states:
            pre_timeout_ctrl = list(states[-1].ctrl)
        else:
            pre_timeout_ctrl = []

    print(f"  Pre-timeout ctrl: {pre_timeout_ctrl}")
    print(f"  Waiting {timeout_ms + 200}ms for timeout...")

    # Stop sending and wait for timeout
    time.sleep((timeout_ms + 200) / 1000.0)

    driver.unsubscribe(sub_id)

    with state_lock:
        if states:
            post_timeout_ctrl = list(states[-1].ctrl)
        else:
            post_timeout_ctrl = []

    print(f"  Post-timeout ctrl: {post_timeout_ctrl}")

    if pre_timeout_ctrl and post_timeout_ctrl:
        pre_nonzero = any(abs(v) > 0.001 for v in pre_timeout_ctrl)
        post_zero = all(abs(v) < 0.001 for v in post_timeout_ctrl)
        if pre_nonzero and post_zero:
            print(f"  PASS: Ctrl zeroed after timeout")
            return True
        elif not pre_nonzero:
            print(f"  SKIP: Pre-timeout ctrl was already zero (model may not have motor actuators)")
            return True
        else:
            print(f"  FAIL: Ctrl not zeroed after timeout")
            return False
    else:
        print(f"  FAIL: Not enough states to verify")
        return False


def main():
    parser = argparse.ArgumentParser(description="Test iMuJoCo control mode decoupling")
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
        print("Failed to connect! Is iMuJoCo running with a model loaded?")
        return 1
    print("Connected!")

    # Send an initial empty control to kickstart state flow
    driver.send_control([])
    time.sleep(0.2)

    results = {}
    results["rate_decoupling"] = test_live_mode_rate_decoupling(driver)
    results["extended_fields"] = test_extended_fields(driver)
    results["ctrl_timeout"] = test_ctrl_timeout(driver)

    driver.disconnect()

    # Summary
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    all_passed = True
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")
        if not passed:
            all_passed = False

    if all_passed:
        print(f"\nAll tests passed!")
    else:
        print(f"\nSome tests failed.")

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())

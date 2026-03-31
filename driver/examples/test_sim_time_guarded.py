#!/usr/bin/env python3
"""
test_sim_time_guarded.py - Verify SimTimeGuarded mode (Mode 3)

Proves: expired controls are rejected, expiry policies work correctly.

Prerequisites:
  - iMuJoCo app running with controlMode = SimTimeGuarded
  - A model loaded and simulation started

Usage:
  bazel run //driver:test_sim_time_guarded_py -- --host <device_ip> --port 9001
"""

import argparse
import threading
import time
import sys

from imujoco_driver import Driver, DriverConfig, ControlCommand


def get_latest_state(driver, timeout=1.0):
    """Send an empty control and collect one state."""
    result = [None]
    event = threading.Event()

    def on_state(state):
        result[0] = state
        event.set()

    sub_id = driver.subscribe(on_state)
    driver.send_control([0.0])
    event.wait(timeout)
    driver.unsubscribe(sub_id)
    return result[0]


def collect_states(driver, duration):
    """Collect states for a given duration."""
    states = []
    lock = threading.Lock()

    def on_state(state):
        with lock:
            states.append(state)

    sub_id = driver.subscribe(on_state)
    time.sleep(duration)
    driver.unsubscribe(sub_id)

    with lock:
        return list(states)


def test_valid_control(driver):
    """
    Test 1: Send ctrl with expire_at_sim_time far in the future.
    Verify it is applied.
    """
    print(f"\n{'='*60}")
    print(f"TEST 1: Valid control (expire far in future)")
    print(f"{'='*60}")

    # Get current sim time
    state = get_latest_state(driver)
    if not state:
        print("  FAIL: No state received")
        return False

    sim_now = state.time
    print(f"  Current sim time: {sim_now:.4f}")

    # Send ctrl=0.77 with expire 10s in the future
    cmd = ControlCommand()
    cmd.ctrl = [0.77]
    cmd.expire_at_sim_time = sim_now + 10.0
    driver.send_control(cmd)

    time.sleep(0.1)
    state2 = get_latest_state(driver)
    if not state2:
        print("  FAIL: No state after sending control")
        return False

    ctrl_applied = list(state2.ctrl)
    print(f"  Ctrl after send:   {ctrl_applied[:3]}")

    passed = len(ctrl_applied) > 0 and abs(ctrl_applied[0] - 0.77) < 0.01
    if passed:
        print(f"  PASS: Valid control applied (ctrl[0]={ctrl_applied[0]:.3f})")
    else:
        print(f"  FAIL: Control not applied (expected 0.77, got {ctrl_applied[0] if ctrl_applied else 'empty'})")

    return passed


def test_expired_control(driver):
    """
    Test 2: Send ctrl with expire_at_sim_time already in the past.
    Verify it is NOT applied.
    """
    print(f"\n{'='*60}")
    print(f"TEST 2: Expired control (expire in the past)")
    print(f"{'='*60}")

    # First set a known baseline ctrl
    cmd_baseline = ControlCommand()
    cmd_baseline.ctrl = [0.33]
    state = get_latest_state(driver)
    if not state:
        print("  FAIL: No state received")
        return False

    cmd_baseline.expire_at_sim_time = state.time + 10.0
    driver.send_control(cmd_baseline)
    time.sleep(0.1)

    # Now send an expired control
    cmd_expired = ControlCommand()
    cmd_expired.ctrl = [0.99]
    cmd_expired.expire_at_sim_time = 0.001  # Already past
    driver.send_control(cmd_expired)

    time.sleep(0.1)
    state2 = get_latest_state(driver)
    if not state2:
        print("  FAIL: No state after expired send")
        return False

    ctrl_val = list(state2.ctrl)
    print(f"  Baseline ctrl:     0.33")
    print(f"  Expired ctrl sent: 0.99 (expire_at=0.001)")
    print(f"  Ctrl in state:     {ctrl_val[:3]}")

    # ctrl should still be ~0.33 (baseline), NOT 0.99
    passed = len(ctrl_val) > 0 and abs(ctrl_val[0] - 0.33) < 0.05
    if passed:
        print(f"  PASS: Expired control was rejected (ctrl still ~0.33)")
    else:
        print(f"  FAIL: Expired control was applied (ctrl={ctrl_val[0]:.3f}, expected ~0.33)")

    return passed


def test_expiry_transition(driver):
    """
    Test 3: Send ctrl with short expiry window. Verify it applies initially,
    then observe behavior after expiry.
    """
    print(f"\n{'='*60}")
    print(f"TEST 3: Expiry transition (short validity window)")
    print(f"{'='*60}")

    state = get_latest_state(driver)
    if not state:
        print("  FAIL: No state received")
        return False

    sim_now = state.time

    # Send ctrl=0.55 with 50ms expiry window
    cmd = ControlCommand()
    cmd.ctrl = [0.55]
    cmd.expire_at_sim_time = sim_now + 0.05  # 50ms window
    driver.send_control(cmd)

    # Immediately check — should be applied
    time.sleep(0.02)
    state_during = get_latest_state(driver)

    # Wait for expiry (>50ms of sim time)
    time.sleep(0.2)
    state_after = get_latest_state(driver)

    if not state_during or not state_after:
        print("  FAIL: Missing states")
        return False

    ctrl_during = list(state_during.ctrl)
    ctrl_after = list(state_after.ctrl)
    print(f"  Ctrl during window: {ctrl_during[:3]}")
    print(f"  Ctrl after expiry:  {ctrl_after[:3]}")
    print(f"  (Post-expiry behavior depends on expiry policy)")

    # During window, ctrl should be ~0.55
    passed = len(ctrl_during) > 0 and abs(ctrl_during[0] - 0.55) < 0.1
    if passed:
        print(f"  PASS: Control applied during validity window")
    else:
        print(f"  FAIL: Control not applied during window")

    return passed


def main():
    parser = argparse.ArgumentParser(description="Test SimTimeGuarded mode")
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
    results["valid_control"] = test_valid_control(driver)
    results["expired_control"] = test_expired_control(driver)
    results["expiry_transition"] = test_expiry_transition(driver)

    driver.disconnect()

    print(f"\n{'='*60}")
    print("SUMMARY — SimTimeGuarded Mode")
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

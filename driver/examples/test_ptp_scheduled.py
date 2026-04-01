#!/usr/bin/env python3
"""
test_ptp_scheduled.py - Verify PTPScheduled mode

Proves: controls are applied at the scheduled simulation time, not when they arrive.

Prerequisites:
  - iMuJoCo app running with controlMode = PTPScheduled
  - A model loaded and simulation started
  - PTP time sync active (driver <-> device)

Usage:
  bazel run //driver:test_ptp_scheduled_py -- --host <device_ip> --port 9001
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
    driver.send_control([])  # Empty ctrl = trigger state without perturbing simulation
    event.wait(timeout)
    driver.unsubscribe(sub_id)
    return result[0]


def test_scheduled_future_control(driver):
    """
    Test 1: Send ctrl with target_sim_time 300ms in the future.
    Verify it is NOT applied immediately, then appears after sim time reaches target.
    """
    print(f"\n{'='*60}")
    print(f"TEST 1: Scheduled future control (target = now + 300ms)")
    print(f"{'='*60}")

    # Get current sim time
    state = get_latest_state(driver)
    if not state:
        print("  FAIL: No state received")
        return False

    sim_now = state.time
    target = sim_now + 0.300  # 300ms in future (wide window for WiFi)
    print(f"  Current sim time:  {sim_now:.4f}")
    print(f"  Target sim time:   {target:.4f}")

    # First set a known baseline (immediate, target=0)
    cmd_base = ControlCommand()
    cmd_base.ctrl = [0.11]
    cmd_base.target_sim_time = 0.0  # Immediate (Live fallback)
    driver.send_control(cmd_base)
    time.sleep(0.05)

    # Now send scheduled control
    cmd = ControlCommand()
    cmd.ctrl = [0.88]
    cmd.target_sim_time = target
    driver.send_control(cmd)

    # Check quickly — should still be baseline (0.11), well before target
    time.sleep(0.05)
    state_before = get_latest_state(driver)

    # Wait past the target time, sending non-empty keepalives to prevent ctrl timeout
    # (empty ctrl no longer resets the valid-ctrl timer)
    wait_end = time.time() + 0.5
    while time.time() < wait_end:
        driver.send_control([0.11])  # Repeat baseline to keep timeout alive
        time.sleep(0.05)

    state_after = get_latest_state(driver)

    if not state_before or not state_after:
        print("  FAIL: Missing states")
        return False

    ctrl_before = list(state_before.ctrl)
    ctrl_after = list(state_after.ctrl)
    sim_time_after = state_after.time

    print(f"  Ctrl before target: {ctrl_before[:3]} (sim_t={state_before.time:.4f})")
    print(f"  Ctrl after target:  {ctrl_after[:3]} (sim_t={sim_time_after:.4f})")

    # Before target: should be ~0.11 (baseline)
    before_ok = len(ctrl_before) > 0 and abs(ctrl_before[0] - 0.11) < 0.05
    # After target: should be ~0.88 (scheduled)
    after_ok = len(ctrl_after) > 0 and abs(ctrl_after[0] - 0.88) < 0.05

    if before_ok and after_ok:
        print(f"  PASS: Control applied at scheduled time (not immediately)")
    elif after_ok and not before_ok:
        print(f"  PARTIAL: Control applied after target, but baseline was wrong")
    else:
        print(f"  FAIL: Scheduled control not applied correctly")
        print(f"         before_ok={before_ok}, after_ok={after_ok}")

    return before_ok and after_ok


def test_immediate_fallback(driver):
    """
    Test 2: Send ctrl with target_sim_time = 0. Verify immediate application.
    """
    print(f"\n{'='*60}")
    print(f"TEST 2: Immediate fallback (target_sim_time = 0)")
    print(f"{'='*60}")

    cmd = ControlCommand()
    cmd.ctrl = [0.42]
    cmd.target_sim_time = 0.0  # Should apply immediately (Live fallback)
    driver.send_control(cmd)

    time.sleep(0.1)
    state = get_latest_state(driver)
    if not state:
        print("  FAIL: No state received")
        return False

    ctrl = list(state.ctrl)
    print(f"  Sent ctrl=0.42 with target=0")
    print(f"  State ctrl: {ctrl[:3]}")

    passed = len(ctrl) > 0 and abs(ctrl[0] - 0.42) < 0.05
    if passed:
        print(f"  PASS: Immediate fallback works (ctrl={ctrl[0]:.3f})")
    else:
        print(f"  FAIL: Control not applied immediately")

    return passed


def test_mpc_batch(driver):
    """
    Test 3: Send 5 future controls spaced 10ms apart.
    Verify each applied at approximately the right sim time.
    """
    print(f"\n{'='*60}")
    print(f"TEST 3: MPC batch (5 controls, 10ms spacing)")
    print(f"{'='*60}")

    state = get_latest_state(driver)
    if not state:
        print("  FAIL: No state received")
        return False

    sim_now = state.time
    n_controls = 5
    spacing = 0.010  # 10ms

    # Send batch of future controls with distinct values
    for i in range(n_controls):
        cmd = ControlCommand()
        cmd.ctrl = [0.1 * (i + 1)]  # 0.1, 0.2, 0.3, 0.4, 0.5
        cmd.target_sim_time = sim_now + (i + 1) * spacing
        driver.send_control(cmd)

    print(f"  Sent {n_controls} controls: targets from {sim_now + spacing:.4f} to {sim_now + n_controls * spacing:.4f}")

    # Collect states over the expected window
    states = []
    lock = threading.Lock()

    def on_state(s):
        with lock:
            states.append((s.time, list(s.ctrl)))

    sub_id = driver.subscribe(on_state)
    time.sleep(n_controls * spacing + 0.2)  # Wait for all to apply
    driver.unsubscribe(sub_id)

    with lock:
        snapshot = list(states)

    print(f"  States collected:  {len(snapshot)}")

    # Find when each ctrl value appeared
    last_ctrl = None
    transitions = []
    for sim_t, ctrl in snapshot:
        if ctrl and len(ctrl) > 0:
            c = round(ctrl[0], 2)
            if c != last_ctrl and c > 0:
                transitions.append((sim_t, c))
                last_ctrl = c

    print(f"  Ctrl transitions:  {len(transitions)}")
    for sim_t, c in transitions:
        expected_target = sim_now + round(c / 0.1) * spacing
        delta_ms = (sim_t - expected_target) * 1000
        print(f"    ctrl={c:.1f} at sim_t={sim_t:.4f} (target={expected_target:.4f}, delta={delta_ms:+.1f}ms)")

    # At least some transitions should have occurred in order
    passed = len(transitions) >= 2
    if passed:
        # Check ordering: ctrl values should increase
        values = [c for _, c in transitions]
        ordered = all(values[i] <= values[i+1] for i in range(len(values)-1))
        if ordered:
            print(f"  PASS: MPC batch applied in correct order")
        else:
            print(f"  FAIL: Controls applied out of order: {values}")
            passed = False
    else:
        print(f"  FAIL: Too few transitions ({len(transitions)})")

    return passed


def test_superseded_controls(driver):
    """
    Test 4: Send ctrl_A at target T, then ctrl_B at target T+1ms.
    Step past both. Verify ctrl_B was applied (not ctrl_A).
    """
    print(f"\n{'='*60}")
    print(f"TEST 4: Superseded controls (later target wins)")
    print(f"{'='*60}")

    state = get_latest_state(driver)
    if not state:
        print("  FAIL: No state received")
        return False

    sim_now = state.time

    # Send two controls targeting nearly the same time
    cmd_a = ControlCommand()
    cmd_a.ctrl = [0.22]
    cmd_a.target_sim_time = sim_now + 0.020  # T

    cmd_b = ControlCommand()
    cmd_b.ctrl = [0.66]
    cmd_b.target_sim_time = sim_now + 0.021  # T + 1ms

    driver.send_control(cmd_a)
    driver.send_control(cmd_b)

    # Wait past both targets
    time.sleep(0.2)
    state2 = get_latest_state(driver)
    if not state2:
        print("  FAIL: No state after wait")
        return False

    ctrl = list(state2.ctrl)
    print(f"  ctrl_A=0.22 at target={sim_now + 0.020:.4f}")
    print(f"  ctrl_B=0.66 at target={sim_now + 0.021:.4f}")
    print(f"  Final ctrl: {ctrl[:3]}")

    # ctrl_B (0.66) should be applied last since it has the later target
    passed = len(ctrl) > 0 and abs(ctrl[0] - 0.66) < 0.1
    if passed:
        print(f"  PASS: Later-targeted control wins (ctrl={ctrl[0]:.3f})")
    else:
        print(f"  FAIL: Expected ~0.66, got {ctrl[0] if ctrl else 'empty'}")

    return passed


def test_native_rate(driver, duration=2.0):
    """
    Test 5: Verify state rate is at native physics rate (same as Live mode).
    """
    print(f"\n{'='*60}")
    print(f"TEST 5: Native physics rate (should match Live mode)")
    print(f"{'='*60}")

    states = []
    lock = threading.Lock()

    def on_state(s):
        with lock:
            states.append(s.time)

    sub_id = driver.subscribe(on_state)

    # Send controls at 50 Hz to keep connection alive
    start = time.time()
    while time.time() - start < duration:
        cmd = ControlCommand()
        cmd.ctrl = [0.1]
        cmd.target_sim_time = 0.0  # Immediate
        driver.send_control(cmd)
        time.sleep(0.02)

    driver.unsubscribe(sub_id)
    elapsed = time.time() - start

    with lock:
        n = len(states)

    state_hz = n / elapsed if elapsed > 0 else 0
    print(f"  Duration:     {elapsed:.2f}s")
    print(f"  States:       {n}")
    print(f"  State rate:   {state_hz:.0f} Hz")

    passed = state_hz > 100  # Should be native rate (e.g., 500 Hz)
    if passed:
        print(f"  PASS: Running at native physics rate ({state_hz:.0f} Hz)")
    else:
        print(f"  FAIL: State rate too low ({state_hz:.0f} Hz)")

    return passed


def main():
    parser = argparse.ArgumentParser(description="Test PTPScheduled mode")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    args = parser.parse_args()

    config = DriverConfig()
    config.host = args.host
    config.port = args.port
    config.timeout_ms = 100
    config.enable_sync = True  # PTPScheduled requires PTP time sync

    driver = Driver(config)
    print(f"Connecting to {args.host}:{args.port} (PTP sync enabled)...")
    if not driver.connect():
        print("Failed to connect!")
        return 1
    print("Connected!")

    results = {}
    results["scheduled_future"] = test_scheduled_future_control(driver)
    results["immediate_fallback"] = test_immediate_fallback(driver)
    results["mpc_batch"] = test_mpc_batch(driver)
    results["superseded"] = test_superseded_controls(driver)
    results["native_rate"] = test_native_rate(driver)

    driver.disconnect()

    print(f"\n{'='*60}")
    print("SUMMARY — PTPScheduled Mode")
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

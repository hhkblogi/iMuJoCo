#!/usr/bin/env python3
"""
test_5min_all_modes.py - 5-minute sustained test for each control mode

Runs each mode for 5 minutes at 50 Hz driver rate and reports:
- State rate (should be ~500 Hz for native physics)
- Real-time ratio (sim_elapsed / wall_elapsed)
- Total states, controls, errors

Requires: simulation running, gRPC available for mode switching.

Usage:
  bazel run //driver:test_5min_all_modes_py -- --host <ip> --port 9001 --grpc_port 8999
"""

import argparse
import math
import os
import subprocess
import sys
import threading
import time

from imujoco_driver import Driver, DriverConfig, ControlCommand


def find_grpc_tester():
    """Find the grpc_tester binary."""
    # Try common locations
    candidates = [
        os.path.join(os.environ.get("BUILD_WORKSPACE_DIRECTORY", ""), "bazel-bin/driver/grpc_tester"),
        "bazel-bin/driver/grpc_tester",
        os.path.expanduser("~/ws-imujoco/imujoco-dev-02/bazel-bin/driver/grpc_tester"),
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    return None


def run_sustained_trial(driver, mode_name, duration=300.0, send_rate=50.0):
    """Run a sustained trial for the given duration. Returns stats dict."""
    states = []
    state_lock = threading.Lock()
    first_state = [None]
    last_state = [None]
    state_count = [0]

    def on_state(state):
        with state_lock:
            state_count[0] += 1
            if first_state[0] is None:
                first_state[0] = (time.time(), state.time)
            last_state[0] = (time.time(), state.time)

    sub_id = driver.subscribe(on_state)

    # Warmup
    driver.send_control([0.0])
    time.sleep(0.2)
    with state_lock:
        first_state[0] = None
        last_state[0] = None
        state_count[0] = 0

    dt = 1.0 / send_rate
    start_wall = time.time()
    send_count = 0
    last_print = start_wall

    try:
        while time.time() - start_wall < duration:
            t = time.time() - start_wall
            ctrl = [0.1 * math.sin(2 * math.pi * 0.5 * t)]

            if mode_name == "PTPScheduled":
                cmd = ControlCommand()
                cmd.ctrl = ctrl
                cmd.target_sim_time = 0.0  # Immediate fallback
                driver.send_control(cmd)
            else:
                driver.send_control(ctrl)

            send_count += 1

            # Progress every 30s
            now = time.time()
            if now - last_print >= 30.0:
                elapsed = now - start_wall
                with state_lock:
                    n = state_count[0]
                rate = n / elapsed if elapsed > 0 else 0
                print(f"    [{elapsed:.0f}s] states={n} ({rate:.0f} Hz) sends={send_count}")
                last_print = now

            time.sleep(dt)
    except KeyboardInterrupt:
        print("    Interrupted")

    driver.unsubscribe(sub_id)
    wall_elapsed = time.time() - start_wall

    with state_lock:
        n = state_count[0]
        fs = first_state[0]
        ls = last_state[0]

    state_hz = n / wall_elapsed if wall_elapsed > 0 else 0
    send_hz = send_count / wall_elapsed if wall_elapsed > 0 else 0

    if fs and ls:
        sim_elapsed = ls[1] - fs[1]
        rt_ratio = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0
    else:
        sim_elapsed = 0.0
        rt_ratio = 0.0

    return {
        "mode": mode_name,
        "duration": wall_elapsed,
        "send_count": send_count,
        "send_hz": send_hz,
        "state_count": n,
        "state_hz": state_hz,
        "sim_elapsed": sim_elapsed,
        "rt_ratio": rt_ratio,
    }


def switch_mode(grpc_target, instance, mode_name):
    """Switch control mode via gRPC tester."""
    tester = find_grpc_tester()
    if not tester:
        print(f"    ERROR: grpc_tester not found. Build it first: bazel build //driver:grpc_tester")
        return False

    mode_map = {
        "Live": "live",
        "PacedReplay": "paced_replay",
        "SimTimeGuarded": "sim_time_guarded",
        "PTPScheduled": "ptp_scheduled",
    }
    mode_str = mode_map.get(mode_name, mode_name.lower())
    result = subprocess.run(
        [tester,
         "--target", grpc_target,
         "--command", "set_mode",
         "--instance", str(instance),
         "--model", mode_str],
        capture_output=True, text=True, timeout=10
    )
    return "success: true" in result.stdout


def main():
    parser = argparse.ArgumentParser(description="5-minute sustained test for all modes")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--grpc_port", type=int, default=8999)
    parser.add_argument("--instance", type=int, default=1)
    parser.add_argument("--duration", type=float, default=300.0, help="Duration per mode (s)")
    args = parser.parse_args()

    grpc_target = f"{args.host}:{args.grpc_port}"

    config = DriverConfig()
    config.host = args.host
    config.port = args.port
    config.timeout_ms = 100

    driver = Driver(config)
    print(f"Connecting to {args.host}:{args.port}...")
    if not driver.connect():
        print("Failed to connect!")
        return 1
    print("Connected!\n")

    modes = ["Live", "PacedReplay", "SimTimeGuarded", "PTPScheduled"]
    results = []

    for mode in modes:
        print(f"{'='*60}")
        print(f"  MODE: {mode} — {args.duration:.0f}s sustained at 50 Hz")
        print(f"{'='*60}")

        if not switch_mode(grpc_target, args.instance, mode):
            print(f"  FAIL: Could not switch to {mode}")
            results.append({"mode": mode, "state_hz": 0, "rt_ratio": 0, "passed": False})
            continue

        r = run_sustained_trial(driver, mode, duration=args.duration)

        # Pass criteria
        if mode == "PacedReplay":
            # PacedReplay: state rate should still be ~500 Hz (sim runs at native rate)
            rate_ok = r["state_hz"] > 200
        else:
            rate_ok = r["state_hz"] > 400  # Native rate ~500 Hz

        rt_ok = abs(r["rt_ratio"] - 1.0) < 0.05  # Within 5% of real-time
        passed = rate_ok and rt_ok

        r["passed"] = passed
        results.append(r)

        print(f"\n  Results:")
        print(f"    Duration:     {r['duration']:.1f}s")
        print(f"    Controls:     {r['send_count']} ({r['send_hz']:.1f} Hz)")
        print(f"    States:       {r['state_count']} ({r['state_hz']:.0f} Hz)")
        print(f"    Sim elapsed:  {r['sim_elapsed']:.2f}s")
        print(f"    RT ratio:     {r['rt_ratio']:.4f}")
        print(f"  {'PASS' if passed else 'FAIL'}")
        print()

        # Brief pause between modes
        time.sleep(1.0)

    driver.disconnect()

    # Summary
    print(f"\n{'='*60}")
    print("FINAL SUMMARY")
    print(f"{'='*60}")
    all_pass = True
    for r in results:
        status = "PASS" if r.get("passed") else "FAIL"
        hz = r.get("state_hz", 0)
        rt = r.get("rt_ratio", 0)
        print(f"  {r['mode']:20s}  {status}  state={hz:.0f} Hz  rt={rt:.4f}")
        if not r.get("passed"):
            all_pass = False

    print(f"\n{'ALL PASS' if all_pass else 'SOME FAILED'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())

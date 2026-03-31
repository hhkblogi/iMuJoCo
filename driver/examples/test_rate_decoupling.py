#!/usr/bin/env python3
"""
test_rate_decoupling.py - Thorough verification of Live mode rate decoupling

The core invariant: simulation ALWAYS steps at native physics rate (1/timestep Hz),
regardless of how fast or slow the driver sends controls.

Tests driver send rates from 10 Hz to 500 Hz and verifies:
  1. State feedback rate ≈ native physics rate (constant across all driver rates)
  2. Simulation time advances at real-time (sim_elapsed ≈ wall_elapsed)
  3. Physics steps/sec is consistent regardless of driver rate
"""

import argparse
import math
import threading
import time
import sys

from imujoco_driver import Driver, DriverConfig


def run_rate_trial(driver, send_rate_hz, duration=3.0):
    """Run a single trial at a given driver send rate. Returns stats dict."""
    states = []
    state_lock = threading.Lock()

    def on_state(state):
        with state_lock:
            states.append((time.time(), state.time, list(state.ctrl)))

    sub_id = driver.subscribe(on_state)

    # Warmup: send one packet to establish connection, wait for state flow
    driver.send_control([0.0])
    time.sleep(0.1)

    # Clear warmup states
    with state_lock:
        states.clear()

    # Send controls at specified rate
    dt = 1.0 / send_rate_hz
    start_wall = time.time()
    send_count = 0

    while time.time() - start_wall < duration:
        t = time.time() - start_wall
        ctrl = [0.1 * math.sin(2 * math.pi * 0.5 * t)]
        driver.send_control(ctrl)
        send_count += 1
        time.sleep(dt)

    # Small delay to collect trailing states
    time.sleep(0.05)

    driver.unsubscribe(sub_id)
    wall_elapsed = time.time() - start_wall

    with state_lock:
        snapshot = list(states)

    n_states = len(snapshot)
    actual_send_hz = send_count / wall_elapsed if wall_elapsed > 0 else 0
    state_hz = n_states / wall_elapsed if wall_elapsed > 0 else 0

    # Compute sim time advancement
    if n_states >= 2:
        sim_start = snapshot[0][1]
        sim_end = snapshot[-1][1]
        sim_elapsed = sim_end - sim_start
        realtime_ratio = sim_elapsed / wall_elapsed if wall_elapsed > 0 else 0
    else:
        sim_elapsed = 0.0
        realtime_ratio = 0.0

    # Estimate physics timestep from state intervals
    if n_states >= 10:
        sim_deltas = [snapshot[i+1][1] - snapshot[i][1] for i in range(min(100, n_states - 1))]
        avg_sim_delta = sum(sim_deltas) / len(sim_deltas) if sim_deltas else 0
        # Filter out zero deltas (multiple states from same step)
        nonzero_deltas = [d for d in sim_deltas if d > 0]
        min_sim_delta = min(nonzero_deltas) if nonzero_deltas else 0
    else:
        avg_sim_delta = 0
        min_sim_delta = 0

    return {
        "send_rate_target": send_rate_hz,
        "send_rate_actual": actual_send_hz,
        "state_rate": state_hz,
        "n_states": n_states,
        "send_count": send_count,
        "wall_elapsed": wall_elapsed,
        "sim_elapsed": sim_elapsed,
        "realtime_ratio": realtime_ratio,
        "avg_sim_delta_ms": avg_sim_delta * 1000,
        "min_sim_delta_ms": min_sim_delta * 1000,
    }


def main():
    parser = argparse.ArgumentParser(description="Thorough rate decoupling verification")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--duration", type=float, default=3.0, help="Duration per trial (s)")
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
    print("Connected!\n")

    # Test at multiple driver send rates
    test_rates = [10, 25, 50, 100, 200, 500]

    print(f"{'Driver Hz':>10} {'Actual Hz':>10} {'State Hz':>10} {'SimTime':>8} {'RT Ratio':>9} {'States':>7}")
    print("-" * 65)

    results = []
    for rate in test_rates:
        r = run_rate_trial(driver, rate, duration=args.duration)
        results.append(r)
        print(f"{r['send_rate_target']:>10} {r['send_rate_actual']:>10.1f} {r['state_rate']:>10.1f} "
              f"{r['sim_elapsed']:>8.2f}s {r['realtime_ratio']:>8.2f}x {r['n_states']:>7}")
        # Brief pause between trials
        time.sleep(0.5)

    driver.disconnect()

    # Analysis
    print(f"\n{'='*65}")
    print("ANALYSIS")
    print(f"{'='*65}")

    # Check 1: State rate should be approximately constant across all driver rates
    state_rates = [r["state_rate"] for r in results if r["n_states"] > 10]
    if state_rates:
        mean_state_rate = sum(state_rates) / len(state_rates)
        max_deviation = max(abs(r - mean_state_rate) for r in state_rates)
        deviation_pct = (max_deviation / mean_state_rate * 100) if mean_state_rate > 0 else 999

        print(f"\n  State rate across trials: mean={mean_state_rate:.0f} Hz, max deviation={deviation_pct:.1f}%")
        if deviation_pct < 20:
            print(f"  PASS: State rate is consistent (±{deviation_pct:.1f}%) — sim rate independent of driver rate")
        else:
            print(f"  FAIL: State rate varies too much (±{deviation_pct:.1f}%) — sim may be coupled to driver")

    # Check 2: Real-time ratio should be ≈ 1.0 for all trials
    rt_ratios = [r["realtime_ratio"] for r in results if r["n_states"] > 10]
    if rt_ratios:
        mean_rt = sum(rt_ratios) / len(rt_ratios)
        max_rt_dev = max(abs(r - 1.0) for r in rt_ratios)

        print(f"\n  Real-time ratio across trials: mean={mean_rt:.3f}, max deviation from 1.0={max_rt_dev:.3f}")
        if max_rt_dev < 0.15:
            print(f"  PASS: Simulation advances at real-time (±{max_rt_dev:.1%})")
        else:
            print(f"  FAIL: Simulation not advancing at real-time (deviation {max_rt_dev:.1%})")

    # Check 3: State rate >> lowest driver rate
    if results:
        lowest_driver = results[0]
        if lowest_driver["state_rate"] > lowest_driver["send_rate_actual"] * 2:
            print(f"\n  PASS: At {lowest_driver['send_rate_target']} Hz driver, state rate "
                  f"({lowest_driver['state_rate']:.0f} Hz) >> driver rate — decoupling confirmed")
        else:
            print(f"\n  FAIL: At {lowest_driver['send_rate_target']} Hz driver, state rate "
                  f"({lowest_driver['state_rate']:.0f} Hz) not much higher — may still be coupled")

    all_pass = (len(state_rates) > 0 and deviation_pct < 20
                and max_rt_dev < 0.15
                and results[0]["state_rate"] > results[0]["send_rate_actual"] * 2)

    print(f"\n{'PASS' if all_pass else 'FAIL'}: Rate decoupling verification {'succeeded' if all_pass else 'failed'}")
    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())

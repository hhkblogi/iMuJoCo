#!/usr/bin/env python3
"""PTP Time Sync Benchmark — 30-Minute Diagnostic Suite.

Comprehensive benchmark for clock synchronization between driver (Mac) and
device (iPad/iPhone). Validates stability for teleoperation with rich
diagnostics covering: convergence, stability, adaptability, and latency.

Usage:
    # Sync-only mode (no control connection, just PTP):
    bazel run //driver:time_sync_benchmark -- --host 192.168.1.42 --port 0

    # Full mode (with state subscription):
    bazel run //driver:time_sync_benchmark -- --host 192.168.1.42 --port 9001

    # Short test:
    bazel run //driver:time_sync_benchmark -- --host 192.168.1.42 --port 0 --duration 1

    # With CSV/JSON output:
    bazel run //driver:time_sync_benchmark -- --host 192.168.1.42 --port 0 --csv sync.csv --json report.json

Prerequisites:
    Load a model on the device and start simulation.
"""

import argparse
import csv
import json
import math
import statistics
import sys
import time

# Force line-buffered output for real-time progress
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

import imujoco_driver


# =============================================================================
# Data collection
# =============================================================================

class SyncSample:
    """A single sync snapshot at a point in time."""
    __slots__ = (
        "elapsed", "locked", "offset_us", "delay_us", "rate_ratio_ppm",
        "exchanges", "jitter_us", "outliers_rejected", "pi_integral",
        "mean_offset_us", "lock_count", "unlock_count",
    )

    def __init__(self, elapsed: float, state: imujoco_driver.ClockSyncState):
        self.elapsed = elapsed
        self.locked = state.locked
        self.offset_us = state.offset_us
        self.delay_us = state.delay_us
        self.rate_ratio_ppm = state.rate_ratio_ppm
        self.exchanges = state.exchanges
        self.jitter_us = state.jitter_us
        self.outliers_rejected = state.outliers_rejected
        self.pi_integral = state.pi_integral
        self.mean_offset_us = state.mean_offset_us
        self.lock_count = state.lock_count
        self.unlock_count = state.unlock_count

    def to_dict(self) -> dict:
        return {s: getattr(self, s) for s in self.__slots__}


class StateSample:
    """A single state reception record."""
    __slots__ = ("elapsed", "sim_time", "step_index", "prediction_error_us")

    def __init__(self, elapsed: float, sim_time: float, step_index: int,
                 prediction_error_us: float | None):
        self.elapsed = elapsed
        self.sim_time = sim_time
        self.step_index = step_index
        self.prediction_error_us = prediction_error_us


# =============================================================================
# Statistics helpers
# =============================================================================

def percentile(data: list[float], p: float) -> float:
    """Compute p-th percentile (0-100)."""
    if not data:
        return 0.0
    s = sorted(data)
    k = (len(s) - 1) * p / 100.0
    f = int(k)
    c = min(f + 1, len(s) - 1)
    d = k - f
    return s[f] + d * (s[c] - s[f])


def linear_regression_slope(xs: list[float], ys: list[float]) -> float:
    """Simple linear regression slope."""
    n = len(xs)
    if n < 2:
        return 0.0
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n
    num = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    den = sum((x - x_mean) ** 2 for x in xs)
    return num / den if den != 0 else 0.0


def fmt_us(v: float) -> str:
    """Format microseconds with commas."""
    return f"{v:,.0f}"


def fmt_ppm(v: float) -> str:
    """Format PPM value."""
    return f"{v:,.0f}"


# =============================================================================
# Window statistics
# =============================================================================

class WindowStats:
    """Aggregated statistics for a time window."""

    def __init__(self, start_min: float, end_min: float,
                 samples: list[SyncSample],
                 state_samples: list[StateSample]):
        self.start_min = start_min
        self.end_min = end_min

        offsets = [s.offset_us for s in samples]
        delays = [s.delay_us for s in samples]
        rates = [s.rate_ratio_ppm for s in samples]
        locked_count = sum(1 for s in samples if s.locked)

        self.offset_mean = statistics.mean(offsets) if offsets else 0
        self.offset_stddev = statistics.stdev(offsets) if len(offsets) > 1 else 0
        self.delay_mean = statistics.mean(delays) if delays else 0
        self.rate_mean = statistics.mean(rates) if rates else 0
        self.lock_pct = 100.0 * locked_count / len(samples) if samples else 0

        pred_errors = [
            s.prediction_error_us for s in state_samples
            if s.prediction_error_us is not None
        ]
        self.pred_error_mean = statistics.mean(pred_errors) if pred_errors else None
        self.states_received = len(state_samples)


# =============================================================================
# Main benchmark
# =============================================================================

def run_benchmark(
    host: str,
    port: int,
    duration_min: float,
    csv_path: str | None,
    json_path: str | None,
    sync_port: int,
    interval_ms: int,
) -> int:
    duration_s = duration_min * 60.0
    interval_s = interval_ms / 1000.0
    sync_only = (port == 0)

    # --- Setup driver ---
    config = imujoco_driver.DriverConfig()
    config.host = host
    config.sync_interval_ms = interval_ms

    if sync_only:
        # Sync-only mode: connect to sync port directly, no control
        config.port = sync_port
        config.sync_port = sync_port
        config.auto_start_receiving = False
    else:
        config.port = port
        config.sync_port = sync_port

    driver = imujoco_driver.Driver(config)

    if not driver.connect():
        print(f"ERROR: Failed to connect to {host}:{config.port}", file=sys.stderr)
        return 1

    # --- Print header ---
    print("=== PTP TIME SYNC BENCHMARK ===")
    print(f"Host: {host}  Sync: {sync_port}  ", end="")
    if sync_only:
        print(f"Mode: sync-only  ", end="")
    else:
        print(f"Control: {port}  ", end="")
    print(f"Duration: {duration_min:.0f}m")
    print()

    # --- State tracking (only in full mode) ---
    sync_samples: list[SyncSample] = []
    state_samples: list[StateSample] = []
    start_time = time.monotonic()

    if not sync_only:
        def on_state(state: imujoco_driver.SimulationState):
            elapsed = time.monotonic() - start_time
            pred_err = None
            sync = driver.get_clock_sync()
            if sync.locked:
                predicted = driver.predict_sim_time()
                if predicted is not None and state.time > 0:
                    pred_err = abs(predicted - state.time) * 1e6
            state_samples.append(StateSample(
                elapsed, state.time, state.step_index, pred_err))

        driver.subscribe(on_state)
        driver.send_control([])  # Trigger state flow

    # --- Convergence tracking ---
    first_lock_time: float | None = None
    first_lock_exchanges: int | None = None
    locked_duration = 0.0
    last_sample_time = start_time
    last_locked = False

    # --- Sampling loop ---
    progress_interval = 30.0  # Print every 30s
    last_progress = -progress_interval  # Print immediately

    try:
        while True:
            time.sleep(interval_s)
            now = time.monotonic()
            elapsed = now - start_time
            if elapsed >= duration_s:
                break

            sync = driver.get_clock_sync()
            sample = SyncSample(elapsed, sync)
            sync_samples.append(sample)

            # Track locked duration
            dt = now - last_sample_time
            if sample.locked:
                locked_duration += dt
            last_sample_time = now

            # Track first lock
            if sample.locked and first_lock_time is None:
                first_lock_time = elapsed
                first_lock_exchanges = sample.exchanges

            last_locked = sample.locked

            # Progress display
            if elapsed - last_progress >= progress_interval:
                last_progress = elapsed
                mins = int(elapsed) // 60
                secs = int(elapsed) % 60
                status = "LOCKED" if sample.locked else "syncing"
                print(
                    f"[{mins:3d}:{secs:02d}] {status:7s}  "
                    f"offset={sample.offset_us}us  "
                    f"delay={sample.delay_us}us  "
                    f"rate={sample.rate_ratio_ppm}ppm  "
                    f"exch={sample.exchanges}  "
                    f"jitter={sample.jitter_us:.1f}us"
                )

    except KeyboardInterrupt:
        print("\nInterrupted — generating report with data collected so far.")

    total_runtime = time.monotonic() - start_time

    # Disconnect cleanly
    driver.disconnect()

    # =========================================================================
    # Generate report
    # =========================================================================

    if not sync_samples:
        print("ERROR: No samples collected.", file=sys.stderr)
        return 1

    # Offset/delay/rate arrays
    offsets = [s.offset_us for s in sync_samples]
    delays = [s.delay_us for s in sync_samples]
    rates = [s.rate_ratio_ppm for s in sync_samples]
    offsets_f = [float(x) for x in offsets]
    delays_f = [float(x) for x in delays]
    rates_f = [float(x) for x in rates]

    # Last sample for PI/outlier stats
    last = sync_samples[-1]

    # Offset drift (linear regression slope over time, µs/min)
    times_min = [s.elapsed / 60.0 for s in sync_samples]
    drift_us_per_min = linear_regression_slope(times_min, offsets_f)

    # Delay spikes (>2x mean)
    delay_mean = statistics.mean(delays_f)
    delay_spikes = sum(1 for d in delays_f if d > 2 * delay_mean)

    # Rate coefficient of variation
    rate_mean = statistics.mean(rates_f) if rates_f else 0
    rate_stddev = statistics.stdev(rates_f) if len(rates_f) > 1 else 0
    rate_cv = abs(rate_stddev / rate_mean) * 100 if rate_mean != 0 else 0

    # Prediction errors
    pred_errors = [
        s.prediction_error_us for s in state_samples
        if s.prediction_error_us is not None
    ]

    # State inter-arrival stats
    state_gaps: list[float] = []
    step_index_gaps: list[int] = []
    for i in range(1, len(state_samples)):
        state_gaps.append(state_samples[i].elapsed - state_samples[i - 1].elapsed)
        idx_gap = state_samples[i].step_index - state_samples[i - 1].step_index
        if idx_gap > 1:
            step_index_gaps.append(idx_gap)

    # 5-minute windows
    window_size_min = 5.0
    windows: list[WindowStats] = []
    w_start = 0.0
    while w_start < total_runtime / 60.0:
        w_end = w_start + window_size_min
        w_sync = [s for s in sync_samples if w_start <= s.elapsed / 60.0 < w_end]
        w_state = [s for s in state_samples if w_start <= s.elapsed / 60.0 < w_end]
        if w_sync:
            windows.append(WindowStats(w_start, w_end, w_sync, w_state))
        w_start = w_end

    # =========================================================================
    # Print report
    # =========================================================================
    print()
    print("=" * 60)
    print("PTP TIME SYNC — BENCHMARK REPORT")
    print(f"Duration: {total_runtime:.1f}s  Samples: {len(sync_samples)}")
    print("=" * 60)

    # --- Convergence ---
    print()
    print("CONVERGENCE")
    if first_lock_time is not None:
        print(f"  Time to lock:        {first_lock_time:.1f}s ({first_lock_exchanges} exchanges)")
    else:
        print("  Time to lock:        NOT LOCKED")
    lock_ratio = 100.0 * locked_duration / total_runtime if total_runtime > 0 else 0
    print(f"  Lock ratio:          {lock_ratio:.1f}% ({locked_duration:.1f}s / {total_runtime:.1f}s)")
    print(f"  Lock/unlock cycles:  {last.lock_count} lock, {last.unlock_count} unlock")

    # --- Offset ---
    print()
    print("OFFSET (us)")
    print(f"  Mean:     {fmt_us(statistics.mean(offsets_f)):>16s}    "
          f"Stddev:   {statistics.stdev(offsets_f) if len(offsets_f) > 1 else 0:.1f}")
    print(f"  p5:       {fmt_us(percentile(offsets_f, 5)):>16s}    "
          f"p50:      {fmt_us(percentile(offsets_f, 50)):>16s}    "
          f"p95:   {fmt_us(percentile(offsets_f, 95))}")
    print(f"  Min:      {fmt_us(min(offsets_f)):>16s}    "
          f"Max:      {fmt_us(max(offsets_f)):>16s}")
    drift_label = "stable" if abs(drift_us_per_min) < 1.0 else ""
    print(f"  Drift:    {drift_us_per_min:+.3f} us/min {drift_label}")

    # --- Delay ---
    print()
    print("DELAY (us)")
    print(f"  Mean:     {fmt_us(delay_mean):>16s}    "
          f"Stddev:   {statistics.stdev(delays_f) if len(delays_f) > 1 else 0:.1f}")
    print(f"  p5:       {fmt_us(percentile(delays_f, 5)):>16s}    "
          f"p50:      {fmt_us(percentile(delays_f, 50)):>16s}    "
          f"p95:   {fmt_us(percentile(delays_f, 95))}    "
          f"p99: {fmt_us(percentile(delays_f, 99))}")
    print(f"  Spikes (>2x mean):  {delay_spikes}")

    # --- Rate ratio ---
    print()
    print("RATE RATIO (ppm)")
    print(f"  Mean:     {fmt_ppm(rate_mean):>16s}    Stddev:   {rate_stddev:.1f}")
    print(f"  Range:    [{fmt_ppm(min(rates_f))}, {fmt_ppm(max(rates_f))}]")
    print(f"  CV:       {rate_cv:.1f}%")

    # --- PI controller ---
    print()
    print("PI CONTROLLER")
    print(f"  Integral:          {last.pi_integral:.1e}")
    total_processed = last.exchanges
    rej_ratio = 100.0 * last.outliers_rejected / total_processed if total_processed > 0 else 0
    print(f"  Outlier ratio:     {rej_ratio:.1f}% ({last.outliers_rejected} / {total_processed})")

    # --- Prediction accuracy ---
    if pred_errors:
        print()
        print("PREDICTION (us)")
        print(f"  Mean error:  {statistics.mean(pred_errors):>8.0f}    "
              f"p50: {percentile(pred_errors, 50):>8.0f}    "
              f"p95: {percentile(pred_errors, 95):>8.0f}    "
              f"p99: {percentile(pred_errors, 99):>8.0f}")

    # --- State channel ---
    if state_samples:
        print()
        print("STATE CHANNEL")
        print(f"  States received:    {len(state_samples)}")
        if state_gaps:
            print(f"  Inter-arrival:      mean={statistics.mean(state_gaps)*1000:.1f}ms  "
                  f"stddev={statistics.stdev(state_gaps)*1000 if len(state_gaps) > 1 else 0:.1f}ms")
        if step_index_gaps:
            print(f"  Step gaps (dropped): {len(step_index_gaps)} "
                  f"(max gap: {max(step_index_gaps)})")
        else:
            print("  Step gaps (dropped): 0")

    # --- Windows ---
    if windows:
        print()
        print("5-MINUTE WINDOWS")
        header = f"  {'Window':>10s} | {'Offset σ':>10s} | {'Delay avg':>10s} | {'Rate avg':>10s} | {'Lock%':>6s}"
        if not sync_only:
            header += f" | {'States':>7s}"
        print(header)
        print("  " + "-" * (len(header) - 2))
        for w in windows:
            row = (f"  {w.start_min:.0f}-{w.end_min:.0f}m"
                   f"     | {w.offset_stddev:>10.1f} | {w.delay_mean:>10.0f} | "
                   f"{w.rate_mean:>10.0f} | {w.lock_pct:>5.1f}%")
            if not sync_only:
                row += f" | {w.states_received:>7d}"
            print(row)

    print("=" * 60)

    # =========================================================================
    # CSV output
    # =========================================================================
    if csv_path:
        fieldnames = list(SyncSample.__slots__)
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for s in sync_samples:
                writer.writerow(s.to_dict())
        print(f"\nCSV raw samples written to: {csv_path}")

    # =========================================================================
    # JSON output
    # =========================================================================
    if json_path:
        report = {
            "config": {
                "host": host,
                "sync_port": sync_port,
                "control_port": port,
                "duration_min": duration_min,
                "interval_ms": interval_ms,
                "sync_only": sync_only,
            },
            "runtime_s": total_runtime,
            "total_samples": len(sync_samples),
            "convergence": {
                "time_to_lock_s": first_lock_time,
                "exchanges_at_lock": first_lock_exchanges,
                "lock_ratio_pct": lock_ratio,
                "locked_duration_s": locked_duration,
                "lock_count": last.lock_count,
                "unlock_count": last.unlock_count,
            },
            "offset_us": {
                "mean": statistics.mean(offsets_f),
                "stddev": statistics.stdev(offsets_f) if len(offsets_f) > 1 else 0,
                "min": min(offsets_f),
                "max": max(offsets_f),
                "p5": percentile(offsets_f, 5),
                "p25": percentile(offsets_f, 25),
                "p50": percentile(offsets_f, 50),
                "p75": percentile(offsets_f, 75),
                "p95": percentile(offsets_f, 95),
                "p99": percentile(offsets_f, 99),
                "drift_us_per_min": drift_us_per_min,
            },
            "delay_us": {
                "mean": delay_mean,
                "stddev": statistics.stdev(delays_f) if len(delays_f) > 1 else 0,
                "min": min(delays_f),
                "max": max(delays_f),
                "p5": percentile(delays_f, 5),
                "p25": percentile(delays_f, 25),
                "p50": percentile(delays_f, 50),
                "p75": percentile(delays_f, 75),
                "p95": percentile(delays_f, 95),
                "p99": percentile(delays_f, 99),
                "spikes_2x": delay_spikes,
            },
            "rate_ratio_ppm": {
                "mean": rate_mean,
                "stddev": rate_stddev,
                "min": min(rates_f),
                "max": max(rates_f),
                "cv_pct": rate_cv,
            },
            "pi_controller": {
                "integral": last.pi_integral,
                "outliers_rejected": last.outliers_rejected,
                "total_exchanges": total_processed,
                "outlier_ratio_pct": rej_ratio,
            },
        }

        if pred_errors:
            report["prediction_us"] = {
                "mean": statistics.mean(pred_errors),
                "p50": percentile(pred_errors, 50),
                "p95": percentile(pred_errors, 95),
                "p99": percentile(pred_errors, 99),
            }

        if state_samples:
            report["state_channel"] = {
                "states_received": len(state_samples),
                "inter_arrival_mean_ms": statistics.mean(state_gaps) * 1000 if state_gaps else None,
                "inter_arrival_stddev_ms": (
                    statistics.stdev(state_gaps) * 1000 if len(state_gaps) > 1 else None
                ),
                "dropped_frames": len(step_index_gaps),
                "max_step_gap": max(step_index_gaps) if step_index_gaps else 0,
            }

        report["windows"] = [
            {
                "start_min": w.start_min,
                "end_min": w.end_min,
                "offset_stddev": w.offset_stddev,
                "delay_mean": w.delay_mean,
                "rate_mean": w.rate_mean,
                "lock_pct": w.lock_pct,
                "states_received": w.states_received,
                "pred_error_mean": w.pred_error_mean,
            }
            for w in windows
        ]

        with open(json_path, "w") as f:
            json.dump(report, f, indent=2)
        print(f"JSON report written to: {json_path}")

    return 0


def main():
    parser = argparse.ArgumentParser(
        description="PTP Time Sync Benchmark — 30-Minute Diagnostic Suite")
    parser.add_argument("--host", required=True, help="Device IP address")
    parser.add_argument("--port", type=int, default=9001,
                        help="Control port (default: 9001). 0 = sync-only mode")
    parser.add_argument("--duration", type=float, default=30.0,
                        help="Test duration in minutes (default: 30)")
    parser.add_argument("--csv", default=None,
                        help="Write raw samples CSV to this path")
    parser.add_argument("--json", default=None,
                        help="Write structured JSON report to this path")
    parser.add_argument("--sync-port", type=int, default=9000,
                        help="Sync port (default: 9000)")
    parser.add_argument("--interval", type=int, default=100,
                        help="Sync sample interval in ms (default: 100)")
    args = parser.parse_args()

    return run_benchmark(
        host=args.host,
        port=args.port,
        duration_min=args.duration,
        csv_path=args.csv,
        json_path=args.json,
        sync_port=args.sync_port,
        interval_ms=args.interval,
    )


if __name__ == "__main__":
    sys.exit(main())

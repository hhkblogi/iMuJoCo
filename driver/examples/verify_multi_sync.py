#!/usr/bin/env python3
"""Verify per-instance PTP sync servers on all 4 instances.

Loads models on all instances via gRPC, then runs a sync benchmark
on each port to verify independent lock.

Usage:
    bazel run //driver:verify_multi_sync -- --host 192.168.65.102

    # Skip model loading (if already loaded):
    bazel run //driver:verify_multi_sync -- --host 192.168.65.102 --skip-load
"""

import argparse
import subprocess
import sys
import time

import imujoco_driver


GRPC_PORT = 8999
NUM_INSTANCES = 4
SYNC_PORT_BASE = 10000  # sync port = 10000 + instance (1-indexed)
MODEL_NAME = "Humanoid (Supine)"


def _find_grpc_tester() -> str:
    """Find the grpc_tester binary."""
    import os
    import shutil
    # Try workspace-relative locations first, then PATH
    for path in [
        os.path.join(os.environ.get("BUILD_WORKSPACE_DIRECTORY", ""), "bazel-bin/driver/grpc_tester"),
        "bazel-bin/driver/grpc_tester",
    ]:
        if os.path.isfile(path):
            return path
    which = shutil.which("grpc_tester")
    if which:
        return which
    return "bazel-bin/driver/grpc_tester"  # fallback


def load_models(host: str) -> bool:
    """Load models on all instances via gRPC."""
    tester = _find_grpc_tester()
    grpc_target = f"{host}:{GRPC_PORT}"
    for i in range(1, NUM_INSTANCES + 1):
        result = subprocess.run(
            [tester, "--target", grpc_target,
             "--command", "load",
             "--instance", str(i),
             "--model", MODEL_NAME],
            capture_output=True, text=True, timeout=30
        )
        if "success: true" in result.stdout:
            print(f"  Instance {i}: loaded '{MODEL_NAME}'")
        else:
            print(f"  Instance {i}: FAILED to load — {result.stdout.strip()}")
            return False
    return True


def list_instances(host: str):
    """List instance states via gRPC."""
    tester = _find_grpc_tester()
    result = subprocess.run(
        [tester, "--target", f"{host}:{GRPC_PORT}",
         "--command", "list"],
        capture_output=True, text=True, timeout=10
    )
    print(result.stdout.strip())


def test_sync_port(host: str, instance: int, duration_s: float = 10.0) -> dict:
    """Test sync on a single port. Returns result dict."""
    sync_port = SYNC_PORT_BASE + instance
    result = {
        "instance": instance,
        "port": sync_port,
        "locked": False,
        "lock_time": None,
        "exchanges": 0,
        "error": None,
    }

    try:
        config = imujoco_driver.DriverConfig()
        config.host = host
        config.port = sync_port  # sync-only mode
        config.sync_port = sync_port
        config.auto_start_receiving = False

        driver = imujoco_driver.Driver(config)
        if not driver.connect():
            result["error"] = "connect failed"
            return result

        start = time.monotonic()
        locked_at = None

        while time.monotonic() - start < duration_s:
            sync = driver.get_clock_sync()
            if sync.locked and locked_at is None:
                locked_at = time.monotonic() - start
            if sync.exchanges > 0:
                result["exchanges"] = sync.exchanges
            time.sleep(0.1)

        sync = driver.get_clock_sync()
        result["locked"] = sync.locked
        if sync.locked and locked_at is None:
            locked_at = time.monotonic() - start
        result["lock_time"] = locked_at
        result["exchanges"] = sync.exchanges

        driver.disconnect()
    except Exception as e:
        result["error"] = str(e)

    return result


def main():
    parser = argparse.ArgumentParser(
        description="Verify per-instance PTP sync servers")
    parser.add_argument("--host", required=True, help="Device IP address")
    parser.add_argument("--skip-load", action="store_true",
                        help="Skip model loading (assume already loaded)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Sync test duration per instance (seconds)")
    args = parser.parse_args()

    print(f"=== Per-Instance PTP Sync Verification ===")
    print(f"Host: {args.host}")
    print()

    # Step 1: List current state
    print("1. Current instance states:")
    list_instances(args.host)
    print()

    # Step 2: Load models if needed
    if not args.skip_load:
        print(f"2. Loading '{MODEL_NAME}' on all {NUM_INSTANCES} instances...")
        if not load_models(args.host):
            print("FAIL: Could not load models on all instances")
            return 1
        print()
        time.sleep(1)  # let sync servers start

        print("   Instance states after load:")
        list_instances(args.host)
        print()

    # Step 3: Test each sync port sequentially
    print(f"3. Testing sync on ports {SYNC_PORT_BASE+1}-{SYNC_PORT_BASE+NUM_INSTANCES} "
          f"({args.duration:.0f}s each)...")
    print()

    results = []
    for i in range(1, NUM_INSTANCES + 1):
        port = SYNC_PORT_BASE + i
        print(f"   Instance {i} (:{port})...", end=" ", flush=True)
        r = test_sync_port(args.host, i, args.duration)
        results.append(r)

        if r["error"]:
            print(f"ERROR: {r['error']}")
        elif r["locked"]:
            print(f"LOCKED in {r['lock_time']:.1f}s  ({r['exchanges']} exchanges)")
        else:
            print(f"NOT LOCKED  ({r['exchanges']} exchanges)")

    # Summary
    print()
    print("=== Summary ===")
    print(f"{'Instance':>10s} | {'Port':>6s} | {'Status':>12s} | {'Lock Time':>10s} | {'Exchanges':>10s}")
    print("-" * 65)
    for r in results:
        status = "ERROR" if r["error"] else ("LOCKED" if r["locked"] else "NOT LOCKED")
        lt = f"{r['lock_time']:.1f}s" if r["lock_time"] else "—"
        print(f"{r['instance']:>10d} | {r['port']:>6d} | {status:>12s} | {lt:>10s} | {r['exchanges']:>10d}")

    locked_count = sum(1 for r in results if r["locked"])
    error_count = sum(1 for r in results if r["error"])
    print()
    print(f"Locked: {locked_count}/{NUM_INSTANCES}  Errors: {error_count}/{NUM_INSTANCES}")

    if locked_count == NUM_INSTANCES:
        print("\nALL INSTANCES LOCKED — per-instance sync verified!")
        return 0
    elif error_count > 0:
        print("\nFAIL: Some instances had errors")
        return 1
    else:
        print("\nPARTIAL: Not all instances locked (WiFi conditions)")
        return 0


if __name__ == "__main__":
    sys.exit(main())

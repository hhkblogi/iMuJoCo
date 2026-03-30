#!/usr/bin/env python3
"""Minimal force data segfault reproducer."""
import sys
import time
import imujoco_driver

config = imujoco_driver.DriverConfig()
config.host = sys.argv[1] if len(sys.argv) > 1 else "192.168.65.102"
config.port = int(sys.argv[2]) if len(sys.argv) > 2 else 9001

driver = imujoco_driver.Driver(config)
if not driver.connect():
    print("Connect failed"); sys.exit(1)
print("Connected", flush=True)

received = [0]
def on_state(state):
    received[0] += 1
    if received[0] == 1:
        # Sanity: total contact force ≈ model_mass * g
        total_force = sum(state.contact_force)
        print(f"  ncon={state.ncon}", flush=True)
        print(f"  contact_force={[round(f,1) for f in state.contact_force]}", flush=True)
        print(f"  total_contact_force={total_force:.1f}N (expect ~400N for humanoid)", flush=True)
        print(f"  qfrc_bias[2] (gravity)={state.qfrc_bias[2]:.1f}N", flush=True)
        print(f"  cfrc_int[0:6]={[round(v,2) for v in state.cfrc_int[0:6]]}", flush=True)

driver.subscribe(on_state)
print("Subscribed, sending...", flush=True)
for i in range(30):
    driver.send_control([])
    time.sleep(0.1)
    if received[0] >= 5:
        break

print(f"Total: {received[0]}", flush=True)
driver.disconnect()
print("Done")

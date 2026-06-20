#!/usr/bin/env python3
"""Quick test: can we talk to RoboClaws at 0x80 and 0x81?"""
import sys
import os

_venv_site = os.path.expanduser('~/ros2_ws/venv/lib/python3.12/site-packages')
if _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

from basicmicro import Basicmicro

PORT = '/dev/ttyAMA0'
BAUD = 38400
ADDRESSES = [0x80, 0x81]

print(f"Opening {PORT} at {BAUD} baud...")
rc = Basicmicro(PORT, BAUD)
if not rc.Open():
    print("ERROR: Could not open serial port")
    sys.exit(1)

for addr in ADDRESSES:
    print(f"\n--- Testing address 0x{addr:02X} ({addr}) ---")
    try:
        result = rc.ReadVersion(addr)
        if result is not None and result[0]:
            print(f"  Version:     {result[1]}")
        else:
            print(f"  ReadVersion: NO RESPONSE")
            continue

        volts = rc.ReadMainBatteryVoltage(addr)
        if volts is not None and volts[0]:
            print(f"  Battery:     {volts[1] / 10.0}V")

        temp = rc.ReadTemp(addr)
        if temp is not None and temp[0]:
            print(f"  Temperature: {temp[1] / 10.0}°C")

    except Exception as e:
        print(f"  ERROR: {e}")

rc.close()
print("\nDone.")

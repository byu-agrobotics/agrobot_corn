#!/usr/bin/env python3
"""Quick test: can we talk to RoboClaws at 0x80 and 0x81?"""
import sys
import os
import time

_venv_site = os.path.expanduser('~/ros2_ws/venv/lib/python3.12/site-packages')
if _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

from basicmicro import Basicmicro

PORT = '/dev/ttyAMA0'
BAUDS = [38400]
ADDRESSES = [0x80, 0x81]

for baud in BAUDS:
    print(f"\n======================================")
    print(f"Trying {PORT} at {baud} baud...")
    rc = Basicmicro(PORT, baud)
    if not rc.Open():
        print("ERROR: Could not open serial port")
        continue
    
    print("Sending forward command (+20% duty)...")
    rc.DutyM1M2(0x80, int(0.2*32767), int(0.2*32767))
    rc.DutyM1M2(0x81, int(0.2*32767), int(0.2*32767))
    time.sleep(2)
    print("Sending reverse command (-20% duty)...")
    rc.DutyM1M2(0x80, int(-0.2*32767), int(-0.2*32767))
    rc.DutyM1M2(0x81, int(-0.2*32767), int(-0.2*32767))
    time.sleep(2)
    print("Stopping motors...")
    rc.DutyM1M2(0x80, 0, 0)
    rc.DutyM1M2(0x81, 0, 0)

    found_any = False
    for addr in ADDRESSES:
        print(f"--- Testing address 0x{addr:02X} ({addr}) ---")
        try:
            result = rc.ReadVersion(addr)
            if result is not None and result[0]:
                print(f"  Version:     {result[1]}")
                found_any = True
                
                volts = rc.ReadMainBatteryVoltage(addr)
                if volts is not None and volts[0]:
                    print(f"  Battery:     {volts[1] / 10.0}V")

                temp = rc.ReadTemp(addr)
                if temp is not None and temp[0]:
                    print(f"  Temperature: {temp[1] / 10.0}°C")
            else:
                print(f"  ReadVersion: NO RESPONSE")

        except Exception as e:
            print(f"  ERROR: {e}")

    rc.close()
    if found_any:
        print(f"\nSUCCESS! Found RoboClaw at {baud} baud.")
        break
else:
    print("\nDone trying all baud rates. No response.")

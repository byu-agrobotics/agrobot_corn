#!/usr/bin/env python3
"""
Set the address of a RoboClaw controller.

Usage: Connect ONLY the RoboClaw you want to re-address, then run:
    python3 set_roboclaw_address.py

This will change it from the default 0x80 (128) to 0x81 (129).
"""
import sys
import os
import time

_venv_site = os.path.expanduser('~/ros2_ws/venv/lib/python3.12/site-packages')
if _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

from basicmicro import Basicmicro

PORT = '/dev/ttyAMA0'
BAUD = 38400
OLD_ADDRESS = 0x80  # current address (default)
NEW_ADDRESS = 0x81  # desired address

print(f"Opening {PORT} at {BAUD} baud...")
rc = Basicmicro(PORT, BAUD)
if not rc.Open():
    print("ERROR: Could not open serial port")
    sys.exit(1)

# Verify we can talk to it at the old address
print(f"Checking for RoboClaw at address 0x{OLD_ADDRESS:02X}...")
try:
    result = rc.ReadVersion(OLD_ADDRESS)
except Exception as e:
    print(f"ERROR: {e}")
    rc.close()
    sys.exit(1)

if result is None or not result[0]:
    print(f"ERROR: No RoboClaw found at address 0x{OLD_ADDRESS:02X}")
    rc.close()
    sys.exit(1)

print(f"Found RoboClaw: {result[1]}")

# Try Method 1: SetAddressMixed (older, more widely supported command)
print(f"\nMethod 1: SetAddressMixed from 0x{OLD_ADDRESS:02X} to 0x{NEW_ADDRESS:02X}...")
try:
    ok = rc.SetAddressMixed(OLD_ADDRESS, NEW_ADDRESS, 0)
    if ok:
        print("  SetAddressMixed succeeded!")
    else:
        print("  SetAddressMixed returned False")
        ok = False
except Exception as e:
    print(f"  SetAddressMixed failed: {e}")
    ok = False

# If Method 1 failed, try Method 2: SetNodeID
if not ok:
    print(f"\nMethod 2: SetNodeID from 0x{OLD_ADDRESS:02X} to 0x{NEW_ADDRESS:02X}...")
    try:
        ok = rc.SetNodeID(OLD_ADDRESS, NEW_ADDRESS)
        if ok:
            print("  SetNodeID succeeded!")
        else:
            print("  SetNodeID returned False")
    except Exception as e:
        print(f"  SetNodeID failed: {e}")
        ok = False

if not ok:
    print("\nERROR: Both methods failed to change the address.")
    print("You may need to use Basicmicro Motion Studio (Windows) to change it via USB.")
    rc.close()
    sys.exit(1)

# Save to NVM so it persists across power cycles
print("\nSaving to NVM...")
time.sleep(0.2)
try:
    nvm_ok = rc.WriteNVM(NEW_ADDRESS)
    if nvm_ok:
        print("  NVM write succeeded!")
    else:
        print("  WARNING: WriteNVM returned False - address may not persist after power cycle")
except Exception as e:
    print(f"  WARNING: WriteNVM failed: {e}")
    print("  The address change may not persist after power cycle")

# Verify the new address works
time.sleep(0.5)
print(f"\nVerifying new address 0x{NEW_ADDRESS:02X}...")
try:
    result = rc.ReadVersion(NEW_ADDRESS)
    if result is not None and result[0]:
        print(f"SUCCESS! RoboClaw is now at address 0x{NEW_ADDRESS:02X} ({NEW_ADDRESS})")
        print(f"Version: {result[1]}")
    else:
        print("WARNING: Could not verify new address. Try power cycling and testing.")
except Exception as e:
    print(f"Verification error: {e}")
    print("Try power cycling the RoboClaw and running test_roboclaw.py")

rc.close()

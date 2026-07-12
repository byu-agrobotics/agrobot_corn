#!/usr/bin/env python3
"""
LCD signal-wire continuity tester (Pi 5 / Ubuntu, native lgpio) — PACED.

The Pi side of this LCD is proven good (power, backlight, GPIO drive, full-frame
SPI TX). If the panel is still black, a SIGNAL wire between the Pi header and the
panel is not making contact. This tool drives each signal to a steady 3.3 V, one
at a time, PAUSING for you to measure with a multimeter AT THE PANEL END.

Meter: DC volts. Black probe on a Pi GND pin (physical 6/9/14/20/25/34/39).
Red probe on the panel's pad for the announced signal.
  ~3.3 V while HIGH  -> wire good
  ~0 V   while HIGH  -> that wire is broken / in the wrong hole  <-- the fault

CS is special: it is GPIO8 / CE0 and is owned by the SPI hardware, so we can't
drive it. A chip-select idles HIGH, so with nothing transferring it should read
~3.3 V at the panel on its own; ~0 V means that wire is broken.

  python3 test_lcd_continuity.py            # paced walk through every signal
  python3 test_lcd_continuity.py rst        # just one signal, held until Enter
"""
import sys
import time

import lgpio

# name -> (BCM, Pi physical pin, panel label, mode)
PINS = [
    ("BL",  26, 37, "BL (known-good reference)", "drive"),
    ("RST",  6, 31, "RST",       "drive"),
    ("DC",  25, 22, "DC",        "drive"),
    ("DIN", 10, 19, "DIN/MOSI",  "drive"),
    ("CLK", 11, 23, "CLK/SCLK",  "drive"),
    ("CS",   8, 24, "CS",        "static"),
]


def _open_chip():
    last = None
    for c in (4, 0):
        try:
            return lgpio.gpiochip_open(c)
        except Exception as e:
            last = e
    raise SystemExit(f"could not open a gpiochip: {last}")


def test_drive(h, name, bcm, phys, label):
    try:
        lgpio.gpio_claim_output(h, bcm, 1)
    except lgpio.error as e:
        print(f"\n>>> {name} GPIO{bcm} (pin {phys}): SKIPPED — {e} "
              f"(pin owned by another function)")
        return
    try:
        print(f"\n>>> {name}: GPIO{bcm} (Pi pin {phys}) -> panel '{label}'  = HIGH (~3.3V)")
        input(f"    Measure the panel '{label}' pad now, then press Enter for next... ")
    finally:
        try:
            lgpio.gpio_write(h, bcm, 0)
            lgpio.gpio_free(h, bcm)
        except Exception:
            pass


def test_static(name, bcm, phys, label):
    print(f"\n>>> {name}: GPIO{bcm} (Pi pin {phys}) -> panel '{label}'  [STATIC — idles HIGH]")
    print("    Can't drive CS (SPI owns CE0). With nothing transferring it should")
    print("    read ~3.3V at the panel if connected; ~0V means the CS wire is broken.")
    input("    Measure the panel 'CS' pad, then press Enter... ")


def main():
    h = _open_chip()
    print("opened gpiochip (native lgpio)")
    print("Make sure no LCD/ROS process is running: pgrep -af lcd")
    print("Black meter probe on a Pi GND pin; red probe at the PANEL end of each wire.\n")

    order = PINS
    if len(sys.argv) > 1:
        want = sys.argv[1].upper()
        order = [p for p in PINS if p[0] == want]
        if not order:
            print("unknown signal; choose:", ", ".join(p[0] for p in PINS))
            lgpio.gpiochip_close(h)
            return
    try:
        for name, bcm, phys, label, mode in order:
            if mode == "static":
                test_static(name, bcm, phys, label)
            else:
                test_drive(h, name, bcm, phys, label)
    except KeyboardInterrupt:
        print("\ninterrupted.")
    finally:
        lgpio.gpiochip_close(h)
    print("\nDone. The DRIVEN wire that read ~0V (or CS that read ~0V) is the culprit.")
    print("After this, re-run:  python3 test_lcd_simple.py   (it re-opens SPI cleanly)")


if __name__ == "__main__":
    main()

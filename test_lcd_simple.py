#!/usr/bin/env python3
"""
Minimal LCD test — no ROS.

  cd ~/ros2_ws && python3 test_lcd_simple.py
  cd ~/ros2_ws && python3 test_lcd_simple.py --spi-hz 1000000 --ce 1
  cd ~/ros2_ws && python3 test_lcd_simple.py --blink-bl

Uses lib/ in this workspace (lcdconfig.py pins: RST/DC/BL and SPI).
"""
from __future__ import annotations

import argparse
import os
import sys
import time
import traceback


def _workspace_root() -> str:
    env = os.environ.get("ROS2_WS")
    if env and os.path.isfile(os.path.join(env, "lib", "LCD_2inch.py")):
        return os.path.abspath(env)
    here = os.path.dirname(os.path.abspath(__file__))
    for root in (here, os.path.dirname(here)):
        if os.path.isfile(os.path.join(root, "lib", "LCD_2inch.py")):
            return root
    return here


def _blink_backlight(bcm_bl: int) -> None:
    """PWM the BL pin — if wiring matches lcdconfig, you should see the backlight pulse."""
    import RPi.GPIO as GPIO

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(bcm_bl, GPIO.OUT)
    pwm = GPIO.PWM(bcm_bl, 500)
    pwm.start(0)
    print(f"Blinking backlight on BCM {bcm_bl} (5 pulses)...")
    for _ in range(5):
        pwm.ChangeDutyCycle(90)
        time.sleep(0.35)
        pwm.ChangeDutyCycle(0)
        time.sleep(0.35)
    pwm.stop()
    GPIO.cleanup([bcm_bl])
    print("Blink done.")


def _patch_lcd_driver(spi_bus: int, spi_ce: int, spi_hz: int, rst: int, dc: int, bl: int) -> None:
    """Inject SPI bus/CE and clock before LCD_2inch is constructed."""
    import lib.lcdconfig as lc

    _orig = lc.RaspberryPi.__init__

    def _patched(
        self,
        spi=None,
        spi_freq=None,
        rst_=rst,
        dc_=dc,
        bl_=bl,
        bl_freq=1000,
        i2c=None,
        i2c_freq=100000,
    ):
        import spidev

        if spi is None:
            spi = spidev.SpiDev(spi_bus, spi_ce)
        if spi_freq is None:
            spi_freq = spi_hz
        return _orig(
            self,
            spi=spi,
            spi_freq=spi_freq,
            rst=rst_,
            dc=dc_,
            bl=bl_,
            bl_freq=bl_freq,
            i2c=i2c,
            i2c_freq=i2c_freq,
        )

    lc.RaspberryPi.__init__ = _patched  # type: ignore[method-assign]


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal Waveshare 2\" SPI LCD test")
    parser.add_argument(
        "--spi-hz",
        type=int,
        default=4_000_000,
        help="SPI max_speed_hz (default 4 MHz; try 1_000_000 if unstable)",
    )
    parser.add_argument(
        "--ce",
        type=int,
        choices=(0, 1),
        default=0,
        help="SPI chip-enable: 0=/dev/spidev0.0 (BCM 8), 1=/dev/spidev0.1 (BCM 7)",
    )
    parser.add_argument("--spi-bus", type=int, default=0, help="SPI bus (usually 0)")
    parser.add_argument("--rst", type=int, default=6, help="BCM reset pin (lcdconfig default)")
    parser.add_argument("--dc", type=int, default=25, help="BCM data/command pin")
    parser.add_argument("--bl", type=int, default=26, help="BCM backlight pin")
    parser.add_argument(
        "--blink-bl",
        action="store_true",
        help="Only blink backlight GPIO (no SPI); verifies BL wiring vs lcdconfig",
    )
    args = parser.parse_args()

    root = _workspace_root()
    sys.path.insert(0, root)

    print("--- LCD hardware smoke test ---")
    print(
        "Stop other programs that use SPI or GPIO 6/8/10/11/25/26 "
        "(e.g. ros2 motor nodes) before testing."
    )
    print("workspace:", root)
    for dev in ("/dev/spidev0.0", "/dev/spidev0.1"):
        print(f"  {dev}: {'exists' if os.path.exists(dev) else 'MISSING'}")

    if args.blink_bl:
        _blink_backlight(args.bl)
        return

    try:
        import lib.lcdconfig as lcdbg

        print("lcdconfig:", lcdbg.__file__)
        print(
            f"pins BCM: RST={args.rst} DC={args.dc} BL={args.bl} | "
            f"SPI{args.spi_bus}.{args.ce} @ {args.spi_hz} Hz"
        )

        _patch_lcd_driver(args.spi_bus, args.ce, args.spi_hz, args.rst, args.dc, args.bl)

        from PIL import Image, ImageDraw

        from lib.LCD_2inch import LCD_2inch

        print("Building LCD_2inch() + Init()...")
        disp = LCD_2inch()
        disp.Init()
        disp.bl_DutyCycle(100)
        print("Init OK; filling full screen (white) via SPI...")
        try:
            disp.clear()
            print("clear() done — if you see a white panel, SPI+DC+CS are OK.")
        except Exception as e:
            print("clear() failed:", e)
        time.sleep(2)

        w, h = 240, 320
        img = Image.new("RGB", (w, h), (200, 30, 30))
        draw = ImageDraw.Draw(img)
        draw.rectangle((10, 10, w - 11, h - 11), outline=(0, 255, 0), width=5)
        draw.text((35, 150), "LCD TEST", fill=(255, 255, 255))

        disp.ShowImage(img)
        print("ShowImage done — expect RED screen, GREEN border, white LCD TEST.")
        print("If still black: try  --ce 1  or  --spi-hz 1000000  or  --blink-bl  (check BL pin).")
        time.sleep(10)

        try:
            disp.module_exit()
        except Exception as e:
            print("module_exit:", e)
    except Exception:
        print("ERROR:")
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

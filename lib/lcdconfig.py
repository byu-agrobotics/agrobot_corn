# /*****************************************************************************
# * | File        :	  epdconfig.py
# * | Author      :   Waveshare team
# * | Function    :   Hardware underlying interface
# * | Info        :
# *----------------
# * | This version:   V1.0
# * | Date        :   2019-06-21
# * | Info        :   
# ******************************************************************************
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

import os
import sys
import time
import spidev
import logging
import numpy as np

# Use RPi.GPIO directly - avoids gpiozero pin factory issues
import RPi.GPIO as GPIO

class RaspberryPi:
    """
    Waveshare 2\" SPI LCD — wiring (BCM = code below; physical = Pi 40-pin header).

    | Signal | Physical | BCM   | In code                          |
    |--------|----------|-------|----------------------------------|
    | VCC    | 1 or 17  | 3.3 V | —                                |
    | GND    | 9 or 25  | GND   | —                                |
    | DIN    | 19       | 10    | MOSI via SpiDev(0, 0)            |
    | CLK    | 23       | 11    | SCLK via SpiDev(0, 0)            |
    | CS     | 24       | 8     | CE0 via SpiDev(0, 0)             |
    | DC     | 22       | 25    | dc=                              |
    | RST    | 31       | 6     | rst=                             |
    | BL     | 37       | 26    | bl=                              |
    """

    # spi_freq: lower Hz helps Dupont/breadboard; raise after wiring is solid.
    def __init__(self,spi=spidev.SpiDev(0,0),spi_freq=4000000,rst = 6,dc = 25,bl = 26,bl_freq=1000,i2c=None,i2c_freq=100000):
        self.np=np
        self.INPUT = False
        self.OUTPUT = True

        self.SPEED  =spi_freq
        self.BL_freq=bl_freq

        # Store pin numbers (RPi.GPIO uses BCM numbering)
        self.RST_PIN = rst
        self.DC_PIN = dc
        self.BL_PIN = bl

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.RST_PIN, GPIO.OUT)
        GPIO.setup(self.DC_PIN, GPIO.OUT)
        GPIO.setup(self.BL_PIN, GPIO.OUT)
        self._bl_pwm = GPIO.PWM(self.BL_PIN, bl_freq)
        self._bl_pwm.start(0)
        
        # Initialize SPI (CE0 = BCM 8 for SpiDev(0, 0); do not GPIO.setup 8/10/11)
        self.SPI = spi
        if self.SPI is not None:
            try:
                self.SPI.bits_per_word = 8
            except (AttributeError, IOError):
                pass
            self.SPI.max_speed_hz = spi_freq
            self.SPI.mode = 0b00

    def gpio_mode(self,Pin,Mode,pull_up = None,active_state = True):
        return Pin  # RPi.GPIO: we just use pin numbers

    def digital_write(self, Pin, value):
        GPIO.output(Pin, GPIO.HIGH if value else GPIO.LOW)

    def digital_read(self, Pin):
        return GPIO.input(Pin)

    def delay_ms(self, delaytime):
        time.sleep(delaytime / 1000.0)

    def gpio_pwm(self,Pin):
        return Pin  # Not used with RPi.GPIO; we use _bl_pwm directly

    def spi_writebyte(self, data):
        if self.SPI is None or not data:
            return
        # writebytes2 avoids extra copies on large pixel bursts (Bookworm / newer spidev)
        if hasattr(self.SPI, "writebytes2") and len(data) > 1:
            try:
                self.SPI.writebytes2(bytes(data))
                return
            except (TypeError, IOError, AttributeError):
                pass
        self.SPI.writebytes(data)

    def bl_DutyCycle(self, duty):
        self._bl_pwm.ChangeDutyCycle(duty)
        
    def bl_Frequency(self,freq):# Hz
        self._bl_pwm.ChangeFrequency(freq)
           
    def module_init(self):
        if self.SPI is not None:
            try:
                self.SPI.bits_per_word = 8
            except (AttributeError, IOError):
                pass
            self.SPI.max_speed_hz = self.SPEED
            self.SPI.mode = 0b00
        return 0

    def module_exit(self):
        logging.debug("spi end")
        if self.SPI!=None :
            self.SPI.close()
        
        logging.debug("gpio cleanup...")
        self.digital_write(self.RST_PIN, 1)
        self.digital_write(self.DC_PIN, 0)
        self._bl_pwm.stop()
        GPIO.cleanup([self.RST_PIN, self.DC_PIN, self.BL_PIN])
        time.sleep(0.001)



'''
if os.path.exists('/sys/bus/platform/drivers/gpiomem-bcm2835'):
    implementation = RaspberryPi()

for func in [x for x in dir(implementation) if not x.startswith('_')]:
    setattr(sys.modules[__name__], func, getattr(implementation, func))
'''

### END OF FILE ###

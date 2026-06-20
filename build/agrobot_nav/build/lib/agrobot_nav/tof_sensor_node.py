#!/usr/bin/env python3
"""
TMF8801 Time-of-Flight Sensor ROS2 Node
========================================

Reads distance measurements from an AMS TMF8801 ToF sensor via I2C
and publishes them as a Float32 on the 'tof_distance' topic.

The current distance is printed to the terminal on every reading.

Wiring (Raspberry Pi → TMF8801):
    Pi Pin 1  (3.3V) → VCC
    Pi Pin 3  (SDA)  → SDA
    Pi Pin 5  (SCL)  → SCL
    Pi Pin 6  (GND)  → GND
"""

import sys
import os
import time

# Ensure venv site-packages are available when launched via ros2 run
_venv_site = os.path.expanduser('~/ros2_ws/venv/lib/python3.12/site-packages')
if _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None


# ── TMF8801 Register Addresses ──────────────────────────────────────────────
TMF8801_ADDR         = 0x41   # Default I2C address

REG_APPID            = 0x00   # Application ID (0x80=bootloader, 0xC0=app)
REG_APPREQID         = 0x02   # Requested application ID
REG_APPREV_MAJOR     = 0x01   # Application revision major
REG_CMD_DATA9        = 0x06   # Command data byte 9 (calibration / config)
REG_CMD_DATA8        = 0x07   # Command data byte 8
REG_CMD_DATA7        = 0x08   # Command data byte 7
REG_CMD_DATA6        = 0x09   # Command data byte 6
REG_CMD_DATA5        = 0x0A   # Command data byte 5
REG_CMD_DATA4        = 0x0B   # Command data byte 4
REG_CMD_DATA3        = 0x0C   # Command data byte 3
REG_CMD_DATA2        = 0x0D   # Command data byte 2
REG_CMD_DATA1        = 0x0E   # Command data byte 1
REG_CMD_DATA0        = 0x0F   # Command data byte 0
REG_COMMAND          = 0x10   # Command register
REG_PREVIOUS         = 0x11   # Previous command
REG_STATUS           = 0x1D   # Measurement status
REG_REGISTER_CONTENTS = 0x1E  # Register contents identifier (expect 0x55)
REG_TID              = 0x1F   # Transaction ID (increments each measurement)
REG_RESULT_NUMBER    = 0x20   # Result number
REG_RESULT_CONFIDENCE = 0x21  # Result confidence (0-63)
REG_DISTANCE_LSB     = 0x22   # Distance low byte (mm)
REG_DISTANCE_MSB     = 0x23   # Distance high byte (mm)
REG_SYSCLK_LSB       = 0x24   # System clock low byte
REG_SYSCLK_MSB       = 0x25   # System clock high byte
REG_ENABLE           = 0xE0   # Enable register
REG_INT_STATUS       = 0xE1   # Interrupt status
REG_ID               = 0xE3   # Device ID
REG_REVID            = 0xE4   # Revision ID

# Command values
CMD_MEASURE          = 0x02   # Start distance measurement
CMD_STOP             = 0xFF   # Stop measurement
CMD_FACTORY_CALIB    = 0x0A   # Factory calibration

# Application IDs
APPID_BOOTLOADER     = 0x80
APPID_APP0           = 0xC0

# Measurement command data for standard distance mode
# Byte layout: [cmd_data9..cmd_data0] = calibration + config
# cmd_data7 (0x08): iterations * 4.8ms (0x04 = ~19ms period)
# cmd_data6 (0x09): repetition period MSB
# cmd_data5 (0x0A): repetition period LSB  (period in ms, 0x0064 = 100ms)
# cmd_data4 (0x0B): 0x00
# cmd_data3 (0x0C): 0x00
# cmd_data2 (0x0D): gpio/interrupt config (0x01 = INT on result)
# cmd_data1 (0x0E): algorithm config (0x23 = combined proximity + distance)
# cmd_data0 (0x0F): GPIO/low power config


class TMF8801Node(Node):
    """ROS2 node that reads from a TMF8801 time-of-flight sensor via I2C."""

    def __init__(self):
        super().__init__('tmf8801_tof_sensor')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', TMF8801_ADDR)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.i2c_bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.i2c_addr = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # Publisher
        self.distance_pub = self.create_publisher(Float32, 'tof_distance', 10)

        # I2C bus handle
        self.bus = None
        self.sensor_ready = False
        self.last_result_number = -1

        # Connect and initialise
        if SMBus is None:
            self.get_logger().error('smbus2 not installed! Run: pip install smbus2')
            return

        if not self._init_sensor():
            self.get_logger().error('TMF8801 initialization failed — check wiring and I2C')
            return

        # Timer to poll distance
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._read_and_publish)
        self.get_logger().info(
            f'TMF8801 node started — publishing on /tof_distance at {self.publish_rate} Hz'
        )

    # ── I2C helpers ──────────────────────────────────────────────────────────

    def _write_reg(self, reg: int, value: int):
        self.bus.write_byte_data(self.i2c_addr, reg, value)

    def _read_reg(self, reg: int) -> int:
        return self.bus.read_byte_data(self.i2c_addr, reg)

    def _read_block(self, reg: int, length: int) -> list:
        return self.bus.read_i2c_block_data(self.i2c_addr, reg, length)

    # ── Sensor initialisation ────────────────────────────────────────────────

    def _init_sensor(self) -> bool:
        """Full TMF8801 initialization sequence."""
        try:
            self.bus = SMBus(self.i2c_bus_num)
        except Exception as e:
            self.get_logger().error(f'Could not open I2C bus {self.i2c_bus_num}: {e}')
            return False

        try:
            # 1. Enable the device
            self._write_reg(REG_ENABLE, 0x01)
            time.sleep(0.1)

            # 2. Wait for device to become ready (ENABLE reads back 0x41)
            for _ in range(50):
                enable_val = self._read_reg(REG_ENABLE)
                if enable_val == 0x41:
                    break
                time.sleep(0.01)
            else:
                self.get_logger().error(
                    f'TMF8801 not ready (ENABLE=0x{enable_val:02X}, expected 0x41)'
                )
                return False

            # 3. Read application ID
            app_id = self._read_reg(REG_APPID)
            self.get_logger().info(f'TMF8801 APP_ID = 0x{app_id:02X}')

            if app_id == APPID_BOOTLOADER:
                # Request switch to application mode
                self.get_logger().info('Sensor in bootloader — requesting APP0...')
                self._write_reg(REG_APPREQID, APPID_APP0)
                time.sleep(0.1)

                # Wait for app to start
                for _ in range(50):
                    app_id = self._read_reg(REG_APPID)
                    if app_id == APPID_APP0:
                        break
                    time.sleep(0.02)
                else:
                    self.get_logger().error(
                        f'Failed to switch to APP0 (APP_ID=0x{app_id:02X})'
                    )
                    return False

            if app_id != APPID_APP0:
                self.get_logger().error(
                    f'Unexpected APP_ID: 0x{app_id:02X} (expected 0xC0)'
                )
                return False

            self.get_logger().info('TMF8801 application running (APP0)')

            # 4. Read device/revision ID
            dev_id = self._read_reg(REG_ID)
            rev_id = self._read_reg(REG_REVID)
            self.get_logger().info(f'Device ID: 0x{dev_id:02X}, Revision: 0x{rev_id:02X}')

            # 5. Configure and start measurement
            self._start_measurement()

            self.sensor_ready = True
            return True

        except Exception as e:
            self.get_logger().error(f'TMF8801 init error: {e}')
            return False

    def _start_measurement(self):
        """Configure and start continuous distance measurement."""
        # Write measurement configuration bytes (cmd_data9 .. cmd_data0)
        # 14 bytes of factory calibration data (all zeros = use ROM defaults)
        # followed by measurement config

        # cmd_data9..cmd_data4: calibration state data (use zeros for defaults)
        for reg in range(REG_CMD_DATA9, REG_CMD_DATA4 + 1):
            self._write_reg(reg, 0x00)

        # cmd_data3 (0x0C): GPIO / interrupt configuration
        self._write_reg(REG_CMD_DATA3, 0x00)

        # cmd_data2 (0x0D): report / interrupt enable (0x01 = enable result int)
        self._write_reg(REG_CMD_DATA2, 0x01)

        # cmd_data1 (0x0E): algorithm state
        #   0x23 = combined short + long range algorithm
        self._write_reg(REG_CMD_DATA1, 0x23)

        # cmd_data0 (0x0F): GPIO / low power config
        self._write_reg(REG_CMD_DATA0, 0x00)

        # Issue the MEASURE command
        self._write_reg(REG_COMMAND, CMD_MEASURE)
        self.get_logger().info('Measurement started (combined proximity + distance mode)')
        time.sleep(0.05)

    # ── Periodic read & publish ──────────────────────────────────────────────

    def _read_and_publish(self):
        """Read the latest distance from the sensor and publish it."""
        if not self.sensor_ready:
            return

        try:
            # Read result block: registers 0x1D – 0x25 (9 bytes)
            data = self._read_block(REG_STATUS, 9)

            # data[0] = 0x1D status
            # data[1] = 0x1E register contents (should be 0x55 for valid)
            # data[2] = 0x1F TID
            # data[3] = 0x20 result_number
            # data[4] = 0x21 confidence (0-63)
            # data[5] = 0x22 distance LSB (mm)
            # data[6] = 0x23 distance MSB (mm)
            # data[7] = 0x24 sys_clk LSB
            # data[8] = 0x25 sys_clk MSB

            register_contents = data[1]
            result_number = data[3]
            confidence = data[4]
            distance_mm = (data[6] << 8) | data[5]

            # Only publish on new results
            if result_number == self.last_result_number:
                return
            self.last_result_number = result_number

            # Check for valid result
            if register_contents != 0x55:
                self.get_logger().debug(
                    f'Invalid result (reg_contents=0x{register_contents:02X})'
                )
                return

            # Publish
            msg = Float32()
            msg.data = float(distance_mm)
            self.distance_pub.publish(msg)

            # Log to terminal
            self.get_logger().info(
                f'Distance: {distance_mm} mm  (confidence: {confidence}/63)'
            )

        except Exception as e:
            self.get_logger().error(f'Read error: {e}')
            # Try to re-start measurement in case it stopped
            try:
                self._write_reg(REG_COMMAND, CMD_MEASURE)
            except Exception:
                pass

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.bus and self.sensor_ready:
            try:
                self._write_reg(REG_COMMAND, CMD_STOP)
                self.get_logger().info('TMF8801 measurement stopped')
            except Exception:
                pass
            try:
                self.bus.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TMF8801Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

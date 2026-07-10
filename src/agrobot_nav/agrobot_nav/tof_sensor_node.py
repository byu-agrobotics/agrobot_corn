#!/usr/bin/env python3
"""
TMF8801 × 8 Time-of-Flight Sensor ROS2 Node (via TCA9548A Mux)
===============================================================

Reads distance measurements from up to 8 TMF8801 ToF sensors connected
through a TCA9548A I2C multiplexer and publishes each as a Float32.

Topics published:
    /tof_distance_0  …  /tof_distance_7   (one per sensor)

The current distances are printed to the terminal on every reading.

Wiring:
    Pi Pin 1  (3.3V)  → TCA9548A VIN, all TMF8801 VCC
    Pi Pin 3  (SDA)   → TCA9548A SDA
    Pi Pin 5  (SCL)   → TCA9548A SCL
    Pi Pin 6  (GND)   → TCA9548A GND, all TMF8801 GND
    TCA9548A RESET    → 3.3V
    TCA9548A A0,A1,A2 → GND  (mux address = 0x70)
    TCA9548A SD0/SC0  → TMF8801 #0 SDA/SCL
    TCA9548A SD1/SC1  → TMF8801 #1 SDA/SCL
       …                  …
    TCA9548A SD7/SC7  → TMF8801 #7 SDA/SCL
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

# TCA9548A defaults
TCA9548A_ADDR        = 0x70   # Default mux address (A0=A1=A2=GND)
NUM_CHANNELS         = 8      # Max channels on the mux


class TMF8801Node(Node):
    """ROS2 node that reads from up to 8 TMF8801 sensors via a TCA9548A mux."""

    def __init__(self):
        super().__init__('tmf8801_tof_sensor')

        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', TMF8801_ADDR)
        self.declare_parameter('mux_address', TCA9548A_ADDR)
        self.declare_parameter('num_sensors', NUM_CHANNELS)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.i2c_bus_num = self.get_parameter('i2c_bus').get_parameter_value().integer_value
        self.i2c_addr = self.get_parameter('i2c_address').get_parameter_value().integer_value
        self.mux_addr = self.get_parameter('mux_address').get_parameter_value().integer_value
        self.num_sensors = self.get_parameter('num_sensors').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # Clamp num_sensors to valid range
        self.num_sensors = max(1, min(NUM_CHANNELS, self.num_sensors))

        # Per-sensor state
        self.sensor_ready = [False] * self.num_sensors
        self.last_result_number = [-1] * self.num_sensors

        # One publisher per sensor: /tof_distance_0 .. /tof_distance_7
        self.distance_pubs = []
        for ch in range(self.num_sensors):
            pub = self.create_publisher(Float32, f'tof_distance_{ch}', 10)
            self.distance_pubs.append(pub)

        # I2C bus handle
        self.bus = None

        # Connect and initialise
        if SMBus is None:
            self.get_logger().error('smbus2 not installed! Run: pip install smbus2')
            return

        if not self._init_all_sensors():
            self.get_logger().warn('Some sensors failed to initialize (see above)')

        # Check we have at least one working sensor
        working = sum(self.sensor_ready)
        if working == 0:
            self.get_logger().error('No sensors initialized — check wiring and I2C')
            return

        # Timer to poll all sensors
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._read_all_and_publish)
        self.get_logger().info(
            f'TMF8801 node started — {working}/{self.num_sensors} sensors active '
            f'at {self.publish_rate} Hz'
        )

    # ── Mux control ──────────────────────────────────────────────────────────

    def _select_channel(self, channel: int):
        """Select a single channel on the TCA9548A mux."""
        self.bus.write_byte(self.mux_addr, 1 << channel)

    def _deselect_all(self):
        """Deselect all mux channels."""
        self.bus.write_byte(self.mux_addr, 0x00)

    # ── I2C helpers (operate on whichever mux channel is active) ─────────────

    def _write_reg(self, reg: int, value: int):
        self.bus.write_byte_data(self.i2c_addr, reg, value)

    def _read_reg(self, reg: int) -> int:
        return self.bus.read_byte_data(self.i2c_addr, reg)

    def _read_block(self, reg: int, length: int) -> list:
        return self.bus.read_i2c_block_data(self.i2c_addr, reg, length)

    # ── Sensor initialisation ────────────────────────────────────────────────

    def _init_all_sensors(self) -> bool:
        """Open the I2C bus and initialize every sensor behind the mux."""
        try:
            self.bus = SMBus(self.i2c_bus_num)
        except Exception as e:
            self.get_logger().error(f'Could not open I2C bus {self.i2c_bus_num}: {e}')
            return False

        for ch in range(self.num_sensors):
            try:
                self._select_channel(ch)
                time.sleep(0.01)
                ok = self._init_single_sensor(ch)
                self.sensor_ready[ch] = ok
            except Exception as e:
                self.get_logger().warn(f'[CH{ch}] Init error: {e}')
                self.sensor_ready[ch] = False

        try:
            self._deselect_all()
        except Exception:
            pass
        return any(self.sensor_ready)

    def _init_single_sensor(self, channel: int) -> bool:
        """Initialize one TMF8801 on the currently-selected mux channel."""
        label = f'[CH{channel}]'

        try:
            # 1. Enable the device
            self._write_reg(REG_ENABLE, 0x01)
            time.sleep(0.1)

            # 2. Wait for device to become ready (ENABLE reads back 0x41)
            enable_val = 0
            for _ in range(50):
                enable_val = self._read_reg(REG_ENABLE)
                if enable_val == 0x41:
                    break
                time.sleep(0.01)
            else:
                self.get_logger().warn(
                    f'{label} Not ready (ENABLE=0x{enable_val:02X}) — no sensor?'
                )
                return False

            # 3. Read application ID
            app_id = self._read_reg(REG_APPID)

            if app_id == APPID_BOOTLOADER:
                self.get_logger().info(f'{label} Bootloader mode — requesting APP0...')
                self._write_reg(REG_APPREQID, APPID_APP0)
                time.sleep(0.1)

                for _ in range(50):
                    app_id = self._read_reg(REG_APPID)
                    if app_id == APPID_APP0:
                        break
                    time.sleep(0.02)
                else:
                    self.get_logger().warn(
                        f'{label} Failed to switch to APP0 (APP_ID=0x{app_id:02X})'
                    )
                    return False

            if app_id != APPID_APP0:
                self.get_logger().warn(
                    f'{label} Unexpected APP_ID: 0x{app_id:02X}'
                )
                return False

            # 4. Configure and start measurement
            self._start_measurement()

            self.get_logger().info(f'{label} TMF8801 initialized and measuring')
            return True

        except OSError:
            # No sensor on this channel — not an error, just skip
            self.get_logger().info(f'{label} No sensor detected — skipping')
            return False
        except Exception as e:
            self.get_logger().warn(f'{label} Init failed: {e}')
            return False

    def _start_measurement(self):
        """Configure and start continuous distance measurement."""
        # cmd_data9..cmd_data4: calibration state data (use zeros for defaults)
        for reg in range(REG_CMD_DATA9, REG_CMD_DATA4 + 1):
            self._write_reg(reg, 0x00)

        # cmd_data3 (0x0C): GPIO / interrupt configuration
        self._write_reg(REG_CMD_DATA3, 0x00)

        # cmd_data2 (0x0D): report / interrupt enable
        self._write_reg(REG_CMD_DATA2, 0x01)

        # cmd_data1 (0x0E): algorithm state (0x23 = combined short + long range)
        self._write_reg(REG_CMD_DATA1, 0x23)

        # cmd_data0 (0x0F): GPIO / low power config
        self._write_reg(REG_CMD_DATA0, 0x00)

        # Issue the MEASURE command
        self._write_reg(REG_COMMAND, CMD_MEASURE)
        time.sleep(0.05)

    # ── Periodic read & publish ──────────────────────────────────────────────

    def _read_all_and_publish(self):
        """Cycle through all active sensors, read distance, and publish."""
        for ch in range(self.num_sensors):
            if not self.sensor_ready[ch]:
                continue

            try:
                self._select_channel(ch)
                self._read_single_sensor(ch)
            except Exception as e:
                self.get_logger().error(f'[CH{ch}] Read cycle error: {e}')

        # Deselect when done to keep bus clean
        try:
            self._deselect_all()
        except Exception:
            pass

    def _read_single_sensor(self, channel: int):
        """Read the latest distance from one sensor and publish it."""
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

            register_contents = data[1]
            result_number = data[3]
            confidence = data[4]
            distance_mm = (data[6] << 8) | data[5]

            # Only publish on new results
            if result_number == self.last_result_number[channel]:
                return
            self.last_result_number[channel] = result_number

            # Check for valid result
            if register_contents != 0x55:
                return

            if confidence < 1:
                return

            # Publish
            msg = Float32()
            msg.data = float(distance_mm)
            self.distance_pubs[channel].publish(msg)

            # Log to terminal
            self.get_logger().info(
                f'[CH{channel}] Distance: {distance_mm} mm  '
                f'(confidence: {confidence}/63)'
            )

        except Exception as e:
            self.get_logger().error(f'[CH{channel}] Read error: {e}')
            # Try to re-start measurement in case it stopped
            try:
                self._write_reg(REG_COMMAND, CMD_MEASURE)
            except Exception:
                pass

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.bus:
            # Stop measurement on each active sensor
            for ch in range(self.num_sensors):
                if not self.sensor_ready[ch]:
                    continue
                try:
                    self._select_channel(ch)
                    self._write_reg(REG_COMMAND, CMD_STOP)
                except Exception:
                    pass

            try:
                self._deselect_all()
            except Exception:
                pass
            try:
                self.bus.close()
            except Exception:
                pass

            self.get_logger().info('All TMF8801 sensors stopped')
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

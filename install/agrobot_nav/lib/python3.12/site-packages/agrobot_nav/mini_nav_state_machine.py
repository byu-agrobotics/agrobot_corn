#!/usr/bin/env python3
"""
PID Navigation State Machine — Square Pattern (5-Point Turn)
=============================================================

Drives straight using PID wall-following on the left side sensors,
then performs a multi-step right turn maneuver designed for a
small arena where there isn't room for a full in-place 90° turn.

Turn sequence:
  1. BACKING_UP          — reverse briefly to clear the wall
  2. TURN_RIGHT_INITIAL  — short timed right turn (no sensor check)
  3. DRIVE_FORWARD_BURST — drive forward a short burst
  4. TURN_RIGHT_SECOND   — another short timed right turn (no sensor check)
  5. DRIVE_FORWARD_BURST_2 — drive forward another short burst
  6. TURN_RIGHT_FINAL    — final timed right turn (no sensor check)
  7. DRIVE_STRAIGHT      — resume PID wall-following

Sensor layout (looking down at the robot):

        FRONT
   ┌──────────────┐
   │ CH0      CH1 │   ← front sensors
   │              │
   │CH2        CH6│   ← side sensors (forward position)
   │              │
   │CH3        CH7│   ← side sensors (rearward position)
   │              │
   │ CH4      CH5 │   ← rear sensors
   └──────────────┘
        REAR

Left wall following uses CH2 + CH3 (averaged).

States:
    IDLE                → motors stopped, waiting for start command
    DRIVE_STRAIGHT      → PID wall-following for drive_duration_s
    BACKING_UP          → reverse briefly after hitting wall
    TURN_RIGHT_INITIAL  → short blind right turn
    DRIVE_FORWARD_BURST → short forward burst to reposition
    TURN_RIGHT_SECOND   → second short blind right turn
    DRIVE_FORWARD_BURST_2 → second short forward burst
    TURN_RIGHT_FINAL    → final short blind right turn
"""

import sys
import os
import time
import threading
from enum import Enum, auto

# Ensure venv site-packages are available
_venv_site = os.path.expanduser('~/ros2_ws/venv/lib/python3.12/site-packages')
if _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tutorial_interfaces.msg import Num

try:
    from basicmicro import Basicmicro
except Exception:
    Basicmicro = None


# ── Constants ────────────────────────────────────────────────────────────────

DUTY_MAX = 32767
NUM_SENSORS = 8

# Sensor channel assignments
CH_FRONT_LEFT  = 0
CH_FRONT_RIGHT = 1
CH_LEFT_FRONT  = 2   # left side, toward the front of the robot
CH_LEFT_REAR   = 3   # left side, toward the rear of the robot
CH_REAR_LEFT   = 4
CH_REAR_RIGHT  = 5
CH_RIGHT_FRONT = 6   # right side, toward the front of the robot
CH_RIGHT_REAR  = 7   # right side, toward the rear of the robot


class State(Enum):
    IDLE = auto()
    DRIVE_ROW_LEFT = auto()
    BACKING_UP = auto()
    TURN_RIGHT_1 = auto()
    DRIVE_BETWEEN_ROWS = auto()
    TURN_RIGHT_2 = auto()
    DRIVE_ROW_RIGHT = auto()


class NavStateMachine(Node):
    """PID navigation state machine with ToF wall-following — square pattern."""

    def __init__(self):
        super().__init__('nav_state_machine')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('wall_distance_mm', 325.0)
        self.declare_parameter('drive_duty_fraction', 0.25)
        self.declare_parameter('turn_duty_fraction', 0.60) # Dirt: 0.60
        self.declare_parameter('backup_duty_fraction', 0.30)
        self.declare_parameter('kp', -60.0)
        self.declare_parameter('ki', 10.00)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('k_angle', 0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('drive_duration_s', 8.0)
        self.declare_parameter('front_stop_distance_mm', 175)
        self.declare_parameter('backup_duration_s', 0.0)
        self.declare_parameter('turn_wall_detect_mm', 60.0) # Dirt: 60.0                  # stop turning when CH_LEFT_FRONT > this
        self.declare_parameter('turn_timeout_s', 2.0)                                     # safety timeout for turn
        self.declare_parameter('turn_min_spin_s', 0.4) # Dirt: 0.6                        # spin at least this long before checking

        self.declare_parameter('between_row_wall_distance_mm', 325.0)
        self.declare_parameter('right_wall_distance_mm', 450.0)
        self.declare_parameter('rear_clearance_mm', 350.0)

        self.wall_setpoint = self.get_parameter('wall_distance_mm').value
        self.between_row_wall_distance = self.get_parameter('between_row_wall_distance_mm').value
        self.right_wall_distance = self.get_parameter('right_wall_distance_mm').value
        self.rear_clearance = self.get_parameter('rear_clearance_mm').value
        self.drive_duty_frac = self.get_parameter('drive_duty_fraction').value
        self.turn_duty_frac = self.get_parameter('turn_duty_fraction').value
        self.backup_duty_frac = self.get_parameter('backup_duty_fraction').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.k_angle = self.get_parameter('k_angle').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.drive_duration = self.get_parameter('drive_duration_s').value
        self.front_stop_distance = self.get_parameter('front_stop_distance_mm').value
        self.backup_duration = self.get_parameter('backup_duration_s').value
        self.turn_wall_detect = self.get_parameter('turn_wall_detect_mm').value
        self.turn_timeout = self.get_parameter('turn_timeout_s').value
        self.turn_min_spin = self.get_parameter('turn_min_spin_s').value

        # ── State ────────────────────────────────────────────────────────────
        self.state = State.IDLE
        self.next_state = State.TURN_RIGHT_1  # State to transition to after BACKING_UP
        self.distances = [0.0] * NUM_SENSORS   # latest reading per channel
        self.sensor_valid = [False] * NUM_SENSORS  # have we received at least one msg?

        # PID state
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = None

        # Timing
        self.phase_start_time = None
        self.leg_count = 0  # how many legs of the square we've completed

        # Logging counter
        self._log_counter = 0

        # Sensor calibration
        self.ch3_offset = 30.0

        # ── Subscribers: ToF sensors ─────────────────────────────────────────
        self.tof_subs = []
        for ch in range(NUM_SENSORS):
            sub = self.create_subscription(
                Float32,
                f'tof_distance_{ch}',
                lambda msg, c=ch: self._tof_callback(c, msg),
                10
            )
            self.tof_subs.append(sub)

        # ── Subscriber: keyboard commands (start/stop) ───────────────────────
        self.cmd_sub = self.create_subscription(
            Num, 'topic', self._cmd_callback, 10
        )

        # ── RoboClaw connection ──────────────────────────────────────────────
        self.controller = None
        self.controller_lock = threading.Lock()
        self.addr_1 = 0x80
        self.addr_2 = 0x81

        if Basicmicro is not None:
            for port in ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/ttyUSB0']:
                if not os.path.exists(port):
                    continue
                try:
                    self.controller = Basicmicro(port, 38400)
                    if self.controller.Open():
                        self.get_logger().info(f'RoboClaw connected on {port}')

                        # Set serial timeout on both RoboClaws (0.5 seconds).
                        # If the Pi crashes and stops sending commands, the
                        # RoboClaws will automatically stop the motors after
                        # this timeout expires. This prevents runaway motors.
                        for addr in [self.addr_1, self.addr_2]:
                            try:
                                self.controller.SetTimeout(addr, 0.5)
                                self.get_logger().info(
                                    f'  RoboClaw 0x{addr:02X}: serial timeout set to 0.5s'
                                )
                            except Exception as e:
                                self.get_logger().warn(
                                    f'  RoboClaw 0x{addr:02X}: failed to set timeout: {e}'
                                )

                        break
                    else:
                        self.controller = None
                except Exception as e:
                    self.get_logger().warn(f'RoboClaw init failed on {port}: {e}')
                    self.controller = None
            if self.controller is None:
                self.get_logger().error('Could not connect to RoboClaw!')
        else:
            self.get_logger().error('basicmicro library not available!')

        # ── Control loop timer ───────────────────────────────────────────────
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('Nav state machine ready (IDLE)')
        self.get_logger().info(
            f'  Wall setpoint: {self.wall_setpoint}mm, '
            f'Drive: {self.drive_duration}s, '
            f'Backup: {self.backup_duration}s'
        )
        self.get_logger().info(
            f'  Turn config: detect_wall<={self.turn_wall_detect}mm, '
            f'min_spin={self.turn_min_spin}s, '
            f'timeout={self.turn_timeout}s'
        )
        self.get_logger().info(
            f'  PID: Kp={self.kp} Ki={self.ki} Kd={self.kd}'
        )
        self.get_logger().info("  Press 'w' to start, 'q' to stop, 'c' to calibrate")


    # ── Callbacks ────────────────────────────────────────────────────────────

    def _tof_callback(self, channel: int, msg: Float32):
        """Store latest ToF reading."""
        self.distances[channel] = msg.data
        self.sensor_valid[channel] = True

    def _cmd_callback(self, msg: Num):
        """Handle keyboard commands: w=start, q=stop, c=calibrate."""
        if msg.num == 1:  # 'w' key
            if self.state == State.IDLE:
                self.get_logger().info('>>> START: IDLE → DRIVE_ROW_LEFT')
                self._reset_pid()
                self.phase_start_time = time.monotonic()
                self.leg_count = 0
                self.state = State.DRIVE_ROW_LEFT
        elif msg.num == 0:  # 'q' key
            if self.state != State.IDLE:
                self.get_logger().info('>>> STOP: → IDLE')
                self._stop_motors()
                self.state = State.IDLE
        elif msg.num == 5:  # 'c' key
            if self.sensor_valid[CH_LEFT_FRONT] and self.sensor_valid[CH_LEFT_REAR]:
                self.ch3_offset = self.distances[CH_LEFT_REAR] - self.distances[CH_LEFT_FRONT]
                self.get_logger().info(f'>>> CALIBRATION COMPLETE: CH3 offset set to {self.ch3_offset:.0f}mm')
            else:
                self.get_logger().warn('>>> CALIBRATION FAILED: Sensors not ready')

    # ── Sensor helpers ───────────────────────────────────────────────────────

    def _front_distance(self) -> float:
        """Average of the two front sensors."""
        vals = []
        if self.sensor_valid[CH_FRONT_LEFT] and self.distances[CH_FRONT_LEFT] > 0:
            vals.append(self.distances[CH_FRONT_LEFT])
        if self.sensor_valid[CH_FRONT_RIGHT] and self.distances[CH_FRONT_RIGHT] > 0:
            vals.append(self.distances[CH_FRONT_RIGHT])
        if not vals:
            return 9999.0
        return sum(vals) / len(vals)

    def _left_wall_distance(self) -> float:
        """Average of the two left-side sensors for wall-following."""
        vals = []
        if self.sensor_valid[CH_LEFT_FRONT] and self.distances[CH_LEFT_FRONT] > 0:
            vals.append(self.distances[CH_LEFT_FRONT])
        if self.sensor_valid[CH_LEFT_REAR] and self.distances[CH_LEFT_REAR] > 0:
            vals.append(self.distances[CH_LEFT_REAR])
        if not vals:
            return self.wall_setpoint  # no reading → assume at setpoint (no correction)
        return sum(vals) / len(vals)

    def _right_wall_distance(self) -> float:
        vals = []
        if self.sensor_valid[CH_RIGHT_FRONT] and self.distances[CH_RIGHT_FRONT] > 0:
            vals.append(self.distances[CH_RIGHT_FRONT])
        if self.sensor_valid[CH_RIGHT_REAR] and self.distances[CH_RIGHT_REAR] > 0:
            vals.append(self.distances[CH_RIGHT_REAR])
        if not vals:
            return self.right_wall_distance
        return sum(vals) / len(vals)

    def _right_wall_angle(self) -> float:
        if (self.sensor_valid[CH_RIGHT_FRONT] and self.distances[CH_RIGHT_FRONT] > 0 and 
            self.sensor_valid[CH_RIGHT_REAR] and self.distances[CH_RIGHT_REAR] > 0):
            return self.distances[CH_RIGHT_FRONT] - self.distances[CH_RIGHT_REAR]
        return 0.0

    def _rear_distance(self) -> float:
        vals = []
        if self.sensor_valid[CH_REAR_LEFT] and self.distances[CH_REAR_LEFT] > 0:
            vals.append(self.distances[CH_REAR_LEFT])
        if self.sensor_valid[CH_REAR_RIGHT] and self.distances[CH_REAR_RIGHT] > 0:
            vals.append(self.distances[CH_REAR_RIGHT])
        if not vals:
            return 9999.0
        return max(vals)  # use max so we don't turn until BOTH have cleared

    def _left_wall_angle(self) -> float:
        """
        Difference between front and rear left sensors.
        Positive means front is further from wall (angled AWAY from wall).
        Negative means front is closer to wall (angled TOWARD wall).
        """
        if (self.sensor_valid[CH_LEFT_FRONT] and self.distances[CH_LEFT_FRONT] > 0 and 
            self.sensor_valid[CH_LEFT_REAR] and self.distances[CH_LEFT_REAR] > 0):
            
            calibrated_rear = self.distances[CH_LEFT_REAR] - self.ch3_offset
            
            return self.distances[CH_LEFT_FRONT] - calibrated_rear
        return 0.0

    # ── PID controller ───────────────────────────────────────────────────────

    def _reset_pid(self):
        """Reset PID state for a fresh start."""
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = time.monotonic()

    def _compute_pid(self, measured: float, angle_diff: float, setpoint: float, is_right_wall: bool = False) -> float:
        """
        Compute PID output for wall-following, incorporating heading angle.

        error = measured - setpoint
          positive → too far from wall → steer left (toward wall)
          negative → too close to wall → steer right (away from wall)
          
        angle_diff = front_sensor - rear_sensor
          positive → angled away from wall → steer left
          negative → angled toward wall → steer right
        """
        now = time.monotonic()
        dt = now - self.pid_last_time if self.pid_last_time else 0.05
        dt = max(dt, 0.001)  # prevent division by zero
        self.pid_last_time = now

        error = measured - setpoint
        if is_right_wall:
            error = -error

        # Proportional
        p_term = self.kp * error

        # Integral (with anti-windup clamp)
        self.pid_integral += error * dt
        self.pid_integral = max(-500.0, min(500.0, self.pid_integral))
        i_term = self.ki * self.pid_integral

        # Derivative (rate of change of distance)
        d_term = self.kd * (error - self.pid_prev_error) / dt
        self.pid_prev_error = error

        # Angle correction (heading)
        if is_right_wall:
            angle_diff = -angle_diff
        angle_term = self.k_angle * angle_diff

        output = p_term + i_term + d_term + angle_term

        # Clamp output to prevent wild swings
        max_correction = 0.3 * DUTY_MAX
        output = max(-max_correction, min(max_correction, output))

        return output

    # ── Motor control ────────────────────────────────────────────────────────

    def _send_motors(self, left_duty: int, right_duty: int):
        """Send duty to both RoboClaws."""
        if self.controller is None:
            return

        # Clamp
        left_duty = max(-DUTY_MAX, min(DUTY_MAX, int(left_duty)))
        right_duty = max(-DUTY_MAX, min(DUTY_MAX, int(right_duty)))

        with self.controller_lock:
            # Invert polarity globally because positive duty drives backwards
            ld = left_duty
            rd = right_duty
            try:
                self.controller.DutyM1M2(self.addr_1, ld, rd)
            except Exception as e:
                self.get_logger().error(f'RoboClaw 1 error: {e}')
            try:
                self.controller.DutyM1M2(self.addr_2, ld, rd)
            except Exception as e:
                self.get_logger().error(f'RoboClaw 2 error: {e}')

    def _stop_motors(self):
        """Immediately stop all motors."""
        self._send_motors(0, 0)

    # ── State machine ────────────────────────────────────────────────────────

    def _control_loop(self):
        """Main control loop — runs at control_rate_hz."""

        if self.state == State.IDLE:
            # Print debug info every 1 second while waiting
            self._log_counter += 1
            if self._log_counter % 10 == 0:
                angle_diff = self._left_wall_angle()
                self.get_logger().info(
                    f'[IDLE] CH2:{self.distances[CH_LEFT_FRONT]:.0f} | '
                    f'CH3_Cal:{(self.distances[CH_LEFT_REAR] - self.ch3_offset):.0f} | '
                    f'CH4:{self.distances[CH_REAR_LEFT]:.0f} | CH5:{self.distances[CH_REAR_RIGHT]:.0f} | '
                    f'CH6:{self.distances[CH_RIGHT_FRONT]:.0f} | CH7:{self.distances[CH_RIGHT_REAR]:.0f} | '
                    f'Ang:{angle_diff:.0f}'
                )

        elif self.state == State.DRIVE_ROW_LEFT:
            self._do_drive_row_left()
            
        elif self.state == State.BACKING_UP:
            self._do_backing_up()

        elif self.state == State.TURN_RIGHT_1:
            self._do_turn_right_1()
            
        elif self.state == State.DRIVE_BETWEEN_ROWS:
            self._do_drive_between_rows()
            
        elif self.state == State.TURN_RIGHT_2:
            self._do_turn_right_2()
            
        elif self.state == State.DRIVE_ROW_RIGHT:
            self._do_drive_row_right()

    def _do_drive_row_left(self):
        """PID wall-following until we reach a wall in front."""

        elapsed = time.monotonic() - self.phase_start_time

        # Check if we've reached the wall
        front_dist = self._front_distance()
        if front_dist <= self.front_stop_distance:
            self.get_logger().info(
                f'Wall reached (front={front_dist:.0f}mm) '
                f'— backing up and turning right'
            )
            self._stop_motors()
            self.phase_start_time = time.monotonic()
            self.state = State.BACKING_UP
            self.next_state = State.TURN_RIGHT_1
            return

        # Check timeout (fallback)
        if elapsed >= self.drive_duration:
            self.get_logger().info(
                f'Drive leg {self.leg_count + 1} complete by timeout ({elapsed:.1f}s) '
                f'— backing up and turning right'
            )
            self._stop_motors()
            self.phase_start_time = time.monotonic()
            self.state = State.BACKING_UP
            self.next_state = State.TURN_RIGHT_1
            return

        # PID wall-following
        left_dist = self._left_wall_distance()
        angle_diff = self._left_wall_angle()
        correction = self._compute_pid(left_dist, angle_diff, self.wall_setpoint)

        # Apply correction differentially:
        #   correction > 0 (too far from wall) → steer left → slow down left, speed up right
        #   correction < 0 (too close to wall) → steer right → speed up left, slow down right
        base_duty = int(self.drive_duty_frac * DUTY_MAX)
        left_duty = int(base_duty - correction)
        right_duty = int(base_duty + correction)

        self._send_motors(left_duty, right_duty)

        # Log periodically (every ~0.5s = every 10th cycle at 20Hz)
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'DRIVE: dist={left_dist:.0f}mm angle={angle_diff:.0f}mm '
                f'front={front_dist:.0f}mm '
                f'correction={correction:.0f} '
                f'duty=({left_duty},{right_duty}) '
                f'time={elapsed:.1f}/{self.drive_duration:.0f}s'
            )

    def _do_backing_up(self):
        """Reverse briefly to clear the wall."""

        elapsed = time.monotonic() - self.phase_start_time

        if elapsed >= self.backup_duration:
            self.get_logger().info(
                f'Backup complete ({elapsed:.1f}s) — turning right'
            )
            self._stop_motors()
            self.phase_start_time = time.monotonic()
            self.state = self.next_state
            return

        # Drive backwards
        backup_duty = -int(self.backup_duty_frac * DUTY_MAX)
        self._send_motors(backup_duty, backup_duty)

    def _do_turn_right_1(self):
        """Turn right until CH7 (left front sensor) reads within
        turn_wall_detect_mm of the wall, meaning we've rotated
        ~90° and are now parallel to the next wall.
        """

        elapsed = time.monotonic() - self.phase_start_time

        # Safety timeout
        if elapsed >= self.turn_timeout:
            self.leg_count += 1
            self.get_logger().warn(
                f'Turn TIMEOUT after {elapsed:.1f}s (leg {self.leg_count}) '
                f'— resuming DRIVE_STRAIGHT'
            )
            self._stop_motors()
            self._reset_pid()
            self.phase_start_time = time.monotonic()
            self.state = State.DRIVE_BETWEEN_ROWS
            return

        # Spin right: left forward, right backward
        turn_duty = int(self.turn_duty_frac * DUTY_MAX)
        self._send_motors(-turn_duty, turn_duty)

        # Don't check sensor until minimum spin time has passed
        if elapsed < self.turn_min_spin:
            return

        # Check CH_LEFT_FRONT — when it detects a wall
        # within the threshold, the robot has turned enough
        if self.sensor_valid[CH_LEFT_FRONT] and self.distances[CH_LEFT_FRONT] > 0:
            side_dist = self.distances[CH_LEFT_FRONT]

            # Log periodically
            self._log_counter += 1
            if self._log_counter % 5 == 0:
                self.get_logger().info(
                    f'TURNING: CH_LEFT_FRONT={side_dist:.0f}mm '
                    f'(stop at >{self.turn_wall_detect:.0f}mm) '
                    f'elapsed={elapsed:.1f}s'
                )

            if side_dist > self.turn_wall_detect:
                self.leg_count += 1
                self.get_logger().info(
                    f'Turn complete! CH_LEFT_FRONT={side_dist:.0f}mm > {self.turn_wall_detect:.0f}mm '
                    f'(leg {self.leg_count}, {elapsed:.1f}s) — resuming DRIVE_STRAIGHT'
                )
                self._stop_motors()
                self._reset_pid()
                self.phase_start_time = time.monotonic()
                self.state = State.DRIVE_STRAIGHT
                return


    def _do_drive_between_rows(self):
        elapsed = time.monotonic() - self.phase_start_time

        # Check if we've cleared the row
        rear_dist = self._rear_distance()
        if rear_dist >= self.rear_clearance and elapsed > 1.0:  # wait 1s so it doesn't trigger instantly
            self.get_logger().info(
                f'Row cleared (rear={rear_dist:.0f}mm) '
                f'— turning right into next row'
            )
            self._stop_motors()
            self.phase_start_time = time.monotonic()
            self.state = State.TURN_RIGHT_2
            return

        # PID wall-following (left wall)
        left_dist = self._left_wall_distance()
        angle_diff = self._left_wall_angle()
        correction = self._compute_pid(left_dist, angle_diff, self.between_row_wall_distance)

        base_duty = int(self.drive_duty_frac * DUTY_MAX)
        left_duty = int(base_duty - correction)
        right_duty = int(base_duty + correction)

        self._send_motors(left_duty, right_duty)

        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'CROSS-DRIVE: L_dist={left_dist:.0f}mm L_angle={angle_diff:.0f}mm '
                f'rear={rear_dist:.0f}mm '
                f'CH4={self.distances[CH_REAR_LEFT]:.0f} CH5={self.distances[CH_REAR_RIGHT]:.0f} '
                f'corr={correction:.0f}'
            )

    def _do_turn_right_2(self):
        elapsed = time.monotonic() - self.phase_start_time

        if elapsed >= self.turn_timeout:
            self.get_logger().warn(f'Turn 2 TIMEOUT ({elapsed:.1f}s) — resuming DRIVE_ROW_RIGHT')
            self._stop_motors()
            self._reset_pid()
            self.phase_start_time = time.monotonic()
            self.state = State.DRIVE_ROW_RIGHT
            return

        turn_duty = int(self.turn_duty_frac * DUTY_MAX)
        self._send_motors(-turn_duty, turn_duty)

        if elapsed < self.turn_min_spin:
            return

        # Check CH_RIGHT_FRONT
        if self.sensor_valid[CH_RIGHT_FRONT] and self.distances[CH_RIGHT_FRONT] > 0:
            side_dist = self.distances[CH_RIGHT_FRONT]
            
            self._log_counter += 1
            if self._log_counter % 5 == 0:
                self.get_logger().info(f'TURN 2: CH_RIGHT_FRONT={side_dist:.0f}mm (stop at >{self.turn_wall_detect:.0f}mm)')

            if side_dist > self.turn_wall_detect:
                self.get_logger().info(f'Turn 2 complete! CH_RIGHT_FRONT={side_dist:.0f}mm — resuming DRIVE_ROW_RIGHT')
                self._stop_motors()
                self._reset_pid()
                self.phase_start_time = time.monotonic()
                self.state = State.DRIVE_ROW_RIGHT
                return

    def _do_drive_row_right(self):
        elapsed = time.monotonic() - self.phase_start_time

        front_dist = self._front_distance()
        if front_dist <= self.front_stop_distance:
            self.get_logger().info('End of 2nd row reached! Stopping.')
            self._stop_motors()
            self.state = State.IDLE
            return

        if elapsed >= self.drive_duration:
            self.get_logger().info('Drive 2 complete by timeout. Stopping.')
            self._stop_motors()
            self.state = State.IDLE
            return

        right_dist = self._right_wall_distance()
        angle_diff = self._right_wall_angle()
        correction = self._compute_pid(right_dist, angle_diff, self.right_wall_distance, is_right_wall=True)

        base_duty = int(self.drive_duty_frac * DUTY_MAX)
        left_duty = int(base_duty - correction)
        right_duty = int(base_duty + correction)

        self._send_motors(left_duty, right_duty)

        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'ROW-RIGHT: R_dist={right_dist:.0f}mm R_angle={angle_diff:.0f}mm '
                f'CH6={self.distances[CH_RIGHT_FRONT]:.0f} CH7={self.distances[CH_RIGHT_REAR]:.0f} '
                f'corr={correction:.0f}'
            )

    # ── Cleanup ──────────────────────────────────────────────────────────────


    def destroy_node(self):
        self._stop_motors()
        if self.controller:
            try:
                self.controller.close()
            except Exception:
                pass
        self.get_logger().info('Nav state machine shut down — motors stopped')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

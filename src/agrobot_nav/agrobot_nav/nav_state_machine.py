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
   │ CH0      CH1 │   ← front sensors (not used for stopping)
   │              │
   │CH7        CH3│   ← side sensors (wall-following)
   │              │
   │CH4        CH5│   ← side sensors (wall-following)
   │              │
   │ CH6      CH2 │   ← rear sensors
   └──────────────┘
        REAR

Left wall following uses CH7 + CH4 (averaged).

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
CH_LEFT_FRONT  = 7   # left side, toward the front of the robot
CH_RIGHT_FRONT = 3   # right side, toward the front of the robot
CH_LEFT_REAR   = 4   # left side, toward the rear of the robot
CH_RIGHT_REAR  = 5   # right side, toward the rear of the robot
CH_REAR_LEFT   = 6
CH_REAR_RIGHT  = 2


class State(Enum):
    IDLE = auto()
    DRIVE_STRAIGHT = auto()
    BACKING_UP = auto()
    TURNING_RIGHT = auto()


class NavStateMachine(Node):
    """PID navigation state machine with ToF wall-following — square pattern."""

    def __init__(self):
        super().__init__('nav_state_machine')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('wall_distance_mm', 100.0)
        self.declare_parameter('drive_duty_fraction', 0.25)
        self.declare_parameter('turn_duty_fraction', 0.50)     # Outer wheels need high power to overcome skid friction
        self.declare_parameter('backup_duty_fraction', 0.30)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('drive_duration_s', 3.0)
        self.declare_parameter('backup_duration_s', 0.6)
        self.declare_parameter('turn_burst_1_s', 0.6)          # Duration of first turn burst
        self.declare_parameter('turn_pause_s', 0.2)            # Duration of pause between bursts
        self.declare_parameter('turn_burst_2_s', 0.4)          # Duration of second turn burst

        self.wall_setpoint = self.get_parameter('wall_distance_mm').value
        self.drive_duty_frac = self.get_parameter('drive_duty_fraction').value
        self.turn_duty_frac = self.get_parameter('turn_duty_fraction').value
        self.backup_duty_frac = self.get_parameter('backup_duty_fraction').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.drive_duration = self.get_parameter('drive_duration_s').value
        self.backup_duration = self.get_parameter('backup_duration_s').value
        self.turn_burst_1 = self.get_parameter('turn_burst_1_s').value
        self.turn_pause = self.get_parameter('turn_pause_s').value
        self.turn_burst_2 = self.get_parameter('turn_burst_2_s').value

        # ── State ────────────────────────────────────────────────────────────
        self.state = State.IDLE
        self.distances = [0.0] * NUM_SENSORS   # latest reading per channel
        self.sensor_valid = [False] * NUM_SENSORS  # have we received at least one msg?

        # PID state
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = None

        # Timing
        self.phase_start_time = None
        self.leg_count = 0  # how many legs of the square we've completed
        self.turn_phase = 0

        # Logging counter
        self._log_counter = 0

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
            f'  PID: Kp={self.kp} Ki={self.ki} Kd={self.kd}'
        )
        self.get_logger().info("  Press 'w' to start, 'q' to stop")


    # ── Callbacks ────────────────────────────────────────────────────────────

    def _tof_callback(self, channel: int, msg: Float32):
        """Store latest ToF reading."""
        self.distances[channel] = msg.data
        self.sensor_valid[channel] = True

    def _cmd_callback(self, msg: Num):
        """Handle keyboard commands: w=start, q=stop."""
        if msg.num == 1:  # 'w' key
            if self.state == State.IDLE:
                self.get_logger().info('>>> START: IDLE → DRIVE_STRAIGHT')
                self._reset_pid()
                self.phase_start_time = time.monotonic()
                self.leg_count = 0
                self.state = State.DRIVE_STRAIGHT
        elif msg.num == 0:  # 'q' key
            if self.state != State.IDLE:
                self.get_logger().info('>>> STOP: → IDLE')
                self._stop_motors()
                self.state = State.IDLE

    # ── Sensor helpers ───────────────────────────────────────────────────────

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

    # ── PID controller ───────────────────────────────────────────────────────

    def _reset_pid(self):
        """Reset PID state for a fresh start."""
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = time.monotonic()

    def _compute_pid(self, measured: float) -> float:
        """
        Compute PID output for wall-following.

        error = measured - setpoint
          positive → too far from wall → steer left (toward wall)
          negative → too close to wall → steer right (away from wall)

        Returns a correction value to apply differentially to motors.
        """
        now = time.monotonic()
        dt = now - self.pid_last_time if self.pid_last_time else 0.05
        dt = max(dt, 0.001)  # prevent division by zero
        self.pid_last_time = now

        error = measured - self.wall_setpoint

        # Proportional
        p_term = self.kp * error

        # Integral (with anti-windup clamp)
        self.pid_integral += error * dt
        self.pid_integral = max(-500.0, min(500.0, self.pid_integral))
        i_term = self.ki * self.pid_integral

        # Derivative
        d_term = self.kd * (error - self.pid_prev_error) / dt
        self.pid_prev_error = error

        output = p_term + i_term + d_term

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
            try:
                self.controller.DutyM1M2(self.addr_1, right_duty, left_duty)
            except Exception as e:
                self.get_logger().error(f'RoboClaw 1 error: {e}')
            try:
                self.controller.DutyM1M2(self.addr_2, right_duty, left_duty)
            except Exception as e:
                self.get_logger().error(f'RoboClaw 2 error: {e}')

    def _stop_motors(self):
        """Immediately stop all motors."""
        self._send_motors(0, 0)

    # ── State machine ────────────────────────────────────────────────────────

    def _control_loop(self):
        """Main control loop — runs at control_rate_hz."""

        if self.state == State.IDLE:
            pass  # do nothing, motors should already be stopped

        elif self.state == State.DRIVE_STRAIGHT:
            self._do_drive_straight()

        elif self.state == State.BACKING_UP:
            self._do_backing_up()

        elif self.state == State.TURNING_RIGHT:
            self._do_turning_right()

    def _do_drive_straight(self):
        """PID wall-following for a fixed duration (drive_duration_s)."""

        elapsed = time.monotonic() - self.phase_start_time

        # Check if we've driven long enough
        if elapsed >= self.drive_duration:
            self.get_logger().info(
                f'Drive leg {self.leg_count + 1} complete ({elapsed:.1f}s) — turning right'
            )
            self._stop_motors()
            self.phase_start_time = time.monotonic()
            self.turn_phase = 0
            self.state = State.TURNING_RIGHT
            return

        # PID wall-following
        left_dist = self._left_wall_distance()
        correction = self._compute_pid(left_dist)

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
                f'DRIVE: left_wall={left_dist:.0f}mm '
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
            self.turn_phase = 0
            self.state = State.TURNING_RIGHT
            return

        # Drive backwards
        backup_duty = -int(self.backup_duty_frac * DUTY_MAX)
        self._send_motors(backup_duty, backup_duty)

    def _do_turning_right(self):
        """Execute a hardcoded two-burst right turn:
        Phase 0: Turn Right (burst 1)
        Phase 1: Pause (stop motors)
        Phase 2: Turn Right (burst 2)
        """
        elapsed = time.monotonic() - self.phase_start_time
        turn_duty = int(self.turn_duty_frac * DUTY_MAX)

        if self.turn_phase == 0:
            self._send_motors(turn_duty, -turn_duty)
            if elapsed >= self.turn_burst_1:
                self._stop_motors()
                self.phase_start_time = time.monotonic()
                self.turn_phase = 1
                self.get_logger().info(f'Turn burst 1 complete ({self.turn_burst_1}s)')
        
        elif self.turn_phase == 1:
            self._stop_motors()
            if elapsed >= self.turn_pause:
                self.phase_start_time = time.monotonic()
                self.turn_phase = 2
                self.get_logger().info(f'Turn pause complete ({self.turn_pause}s)')

        elif self.turn_phase == 2:
            self._send_motors(turn_duty, -turn_duty)
            if elapsed >= self.turn_burst_2:
                self._stop_motors()
                self._reset_pid()
                self.phase_start_time = time.monotonic()
                self.state = State.DRIVE_STRAIGHT
                self.leg_count += 1
                self.get_logger().info(
                    f'Turn burst 2 complete ({self.turn_burst_2}s) — resuming DRIVE_STRAIGHT (leg {self.leg_count})'
                )
                return

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

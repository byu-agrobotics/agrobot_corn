#!/usr/bin/env python3
"""
Navigation State Machine — Corn Row Arena
==========================================

The robot starts in the top-left corner of a 96"×96" arena and must:
  1. Drive from the corner to the first corn row entrance
  2. Turn 90° into the row
  3. Drive through the row with PID wall-following
  4. At the end of the row, perform a U-turn (two 90° turns with a drive between)
  5. Repeat for all rows

States:
    IDLE              → motors stopped, waiting for 'w' command
    NAVIGATE_TO_ROW   → drive from corner to first row entrance (left-wall follow)
    EXECUTE_TURN      → open-loop timed 90° turn burst
    ALIGN_TO_ROW      → PID-based squareness correction using back sensors
    DRIVE_ROW         → PID wall-following through corn row
    APPROACH_WALL     → PID-controlled deceleration at end of row
    DRIVE_TO_NEXT_ROW → drive between U-turn legs (back-sensor distance tracking)

Sensor layout (looking down at the robot):

          FRONT
     ┌──────────────┐
     │ CH0      CH1 │   ← front sensors
     │              │
     │CH2        CH6│   ← left / right side, forward position
     │              │
     │CH3        CH7│   ← left / right side, rearward position
     │              │
     │ CH4      CH5 │   ← back sensors
     └──────────────┘
          REAR

Wall following alternates:
    Row 1 (odd)  → follow RIGHT wall
    Row 2 (even) → follow LEFT wall
    Row 3 (odd)  → follow RIGHT wall ...

Turn direction:
    After odd rows  → turn RIGHT
    After even rows → turn LEFT
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

from std_srvs.srv import Trigger # added so top level FSM can communicate

try:
    from basicmicro import Basicmicro
except Exception:
    Basicmicro = None


# ── Constants ────────────────────────────────────────────────────────────────

DUTY_MAX = 32767
NUM_SENSORS = 8
PID_OUTPUT_LIMIT = int(0.3 * DUTY_MAX)   # ~9830, max PID correction

# Sensor channel assignments (corrected layout)
CH_FRONT_LEFT  = 0
CH_FRONT_RIGHT = 1
CH_LEFT_FRONT  = 2   # left side, toward the front of the robot
CH_LEFT_REAR   = 3   # left side, toward the rear of the robot
CH_BACK_LEFT   = 4
CH_BACK_RIGHT  = 5
CH_RIGHT_FRONT = 6   # right side, toward the front of the robot
CH_RIGHT_REAR  = 7   # right side, toward the rear of the robot


# ── PID Controller ───────────────────────────────────────────────────────────

class PIDController:
    """Simple PID controller with anti-windup clamping.

    Gains are in units of 'duty per mm' — e.g. kp=100 means
    100 duty-cycle units of correction per 1 mm of sensor error.
    """

    def __init__(self, kp: float, ki: float, kd: float,
                 output_limit: float = PID_OUTPUT_LIMIT):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        """Compute PID output.  *error* is in mm, output in duty units."""
        self.integral += error * dt
        # Anti-windup clamp
        self.integral = max(-self.output_limit,
                            min(self.output_limit, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        output = (self.kp * error
                  + self.ki * self.integral
                  + self.kd * derivative)
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


# ── State Enum ───────────────────────────────────────────────────────────────

class State(Enum):
    IDLE              = auto()
    NAVIGATE_TO_ROW   = auto()
    EXECUTE_TURN      = auto()
    ALIGN_TO_ROW      = auto()
    DRIVE_ROW         = auto()
    APPROACH_WALL     = auto()
    DRIVE_TO_NEXT_ROW = auto()


# ── Main Node ────────────────────────────────────────────────────────────────

class NavStateMachine(Node):
    """PID navigation state machine for corn-row arena."""

    def __init__(self):
        super().__init__('nav_state_machine')

        # ── Parameters ───────────────────────────────────────────────────
        # Duty fractions
        self.declare_parameter('drive_duty_fraction', 0.25)
        self.declare_parameter('turn_duty_fraction', 0.55)
        self.declare_parameter('control_rate_hz', 10.0)

        # Phase durations / distances
        self.declare_parameter('turn_burst_s', 0.8)
        self.declare_parameter('approach_stop_mm', 80.0)
        self.declare_parameter('row_wall_follow_mm', 100.0)
        self.declare_parameter('row_approach_mm', 300.0)
        self.declare_parameter('next_row_spacing_mm', 178.0)

        # Alignment tolerances
        self.declare_parameter('align_heading_tol_mm', 10.0)
        self.declare_parameter('align_lateral_tol_mm', 15.0)
        self.declare_parameter('align_hold_time_s', 0.5)

        # Heading PID (back-sensor squareness / side-sensor heading)
        # Units: duty per mm — e.g. 100 means 10mm error → 1000 duty (~3%)
        self.declare_parameter('heading_kp', 100.0)
        self.declare_parameter('heading_ki', 0.0)
        self.declare_parameter('heading_kd', 20.0)

        # Lateral PID (side-wall distance)
        self.declare_parameter('lateral_kp', 80.0)
        self.declare_parameter('lateral_ki', 0.0)
        self.declare_parameter('lateral_kd', 15.0)

        # Front-distance PID (approach deceleration)
        self.declare_parameter('front_kp', 50.0)
        self.declare_parameter('front_ki', 0.0)
        self.declare_parameter('front_kd', 10.0)

        # ── Read parameters ──────────────────────────────────────────────
        self.drive_duty_frac   = self.get_parameter('drive_duty_fraction').value
        self.turn_duty_frac    = self.get_parameter('turn_duty_fraction').value
        self.control_rate      = self.get_parameter('control_rate_hz').value

        self.turn_burst        = self.get_parameter('turn_burst_s').value
        self.approach_stop     = self.get_parameter('approach_stop_mm').value
        self.row_wall_follow   = self.get_parameter('row_wall_follow_mm').value
        self.row_approach      = self.get_parameter('row_approach_mm').value
        self.next_row_spacing  = self.get_parameter('next_row_spacing_mm').value

        self.align_heading_tol = self.get_parameter('align_heading_tol_mm').value
        self.align_lateral_tol = self.get_parameter('align_lateral_tol_mm').value
        self.align_hold_time   = self.get_parameter('align_hold_time_s').value

        # ── PID controllers ──────────────────────────────────────────────
        self.heading_pid = PIDController(
            self.get_parameter('heading_kp').value,
            self.get_parameter('heading_ki').value,
            self.get_parameter('heading_kd').value,
        )
        self.lateral_pid = PIDController(
            self.get_parameter('lateral_kp').value,
            self.get_parameter('lateral_ki').value,
            self.get_parameter('lateral_kd').value,
        )
        self.front_pid = PIDController(
            self.get_parameter('front_kp').value,
            self.get_parameter('front_ki').value,
            self.get_parameter('front_kd').value,
        )

        # ── State ────────────────────────────────────────────────────────
        self.state = State.IDLE
        self.distances = [0.0] * NUM_SENSORS
        self.sensor_valid = [False] * NUM_SENSORS

        # Navigation tracking
        self.row_count = 0            # current row (1-indexed once navigating)
        self.u_turn_leg = 0           # 0=none, 1=first turn done, 2=second turn done
        self.follow_right_wall = True # True for odd rows, False for even
        self.turn_direction = 1       # +1 = right, -1 = left

        # Timing
        self.phase_start_time = None
        self.last_control_time = None
        self.align_stable_since = None

        # DRIVE_TO_NEXT_ROW distance tracking
        self.nav_start_back_distance = None

        # Logging counter
        self._log_counter = 0

        # ── Subscribers: ToF sensors ─────────────────────────────────────
        self.tof_subs = []
        for ch in range(NUM_SENSORS):
            sub = self.create_subscription(
                Float32,
                f'tof_distance_{ch}',
                lambda msg, c=ch: self._tof_callback(c, msg),
                10
            )
            self.tof_subs.append(sub)

        # ── Subscriber: keyboard commands (start/stop) ───────────────────
        self.cmd_sub = self.create_subscription(
            Num, 'topic', self._cmd_callback, 10
        )

        # ── RoboClaw connection ──────────────────────────────────────────
        self.controller = None
        self.controller_lock = threading.Lock()
        self.addr_1 = 0x80
        self.addr_2 = 0x81

        if Basicmicro is not None:
            for port in ['/dev/ttyAMA10', '/dev/ttyAMA0', '/dev/ttyACM0', '/dev/ttyUSB0']:
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

        # ── Control loop timer ───────────────────────────────────────────
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self._control_loop)

        self.get_logger().info('Nav state machine ready (IDLE)')
        self.get_logger().info(
            f'  Wall follow: {self.row_wall_follow:.0f}mm, '
            f'Approach stop: {self.approach_stop:.0f}mm, '
            f'Turn burst: {self.turn_burst}s'
        )
        self.get_logger().info("  Press 'w' to start, 'q' to stop")

    # ══════════════════════════════════════════════════════════════════════
    #  Callbacks
    # ══════════════════════════════════════════════════════════════════════

    def _tof_callback(self, channel: int, msg: Float32):
        """Store latest ToF reading."""
        self.distances[channel] = msg.data
        self.sensor_valid[channel] = True

    def _cmd_callback(self, msg: Num):
        """Handle keyboard commands: w=start, q=stop."""
        if msg.num == 1:  # 'w' key
            if self.state == State.IDLE:
                self.get_logger().info('>>> START: IDLE → NAVIGATE_TO_ROW')
                self._reset_navigation()
                self.phase_start_time = time.monotonic()
                self.last_control_time = time.monotonic()
                self.state = State.NAVIGATE_TO_ROW
        elif msg.num == 0:  # 'q' key
            if self.state != State.IDLE:
                self.get_logger().info('>>> STOP: → IDLE')
                self._stop_motors()
                self.state = State.IDLE

    # ══════════════════════════════════════════════════════════════════════
    #  Sensor Helpers
    # ══════════════════════════════════════════════════════════════════════

    def _get_sensor(self, ch: int):
        """Return sensor reading in mm, or None if invalid."""
        if self.sensor_valid[ch] and self.distances[ch] > 0:
            return self.distances[ch]
        return None

    def _front_distance(self) -> float:
        """Average of both front sensors.  Returns 9999 if no data."""
        vals = [v for v in [self._get_sensor(CH_FRONT_LEFT),
                            self._get_sensor(CH_FRONT_RIGHT)]
                if v is not None]
        return sum(vals) / len(vals) if vals else 9999.0

    def _back_distance(self) -> float:
        """Average of both back sensors.  Returns 0 if no data."""
        vals = [v for v in [self._get_sensor(CH_BACK_LEFT),
                            self._get_sensor(CH_BACK_RIGHT)]
                if v is not None]
        return sum(vals) / len(vals) if vals else 0.0

    def _back_heading_error(self) -> float:
        """Back-sensor difference: CH4 - CH5.
        Near zero = squared up to the wall behind.
        Positive = left-back farther → need to spin right to equalise.
        """
        left = self._get_sensor(CH_BACK_LEFT)
        right = self._get_sensor(CH_BACK_RIGHT)
        if left is None or right is None:
            return 0.0
        return left - right

    def _side_sensors(self, side: str):
        """Return (front_val, rear_val) for the given side, or None."""
        if side == 'right':
            return (self._get_sensor(CH_RIGHT_FRONT),
                    self._get_sensor(CH_RIGHT_REAR))
        else:
            return (self._get_sensor(CH_LEFT_FRONT),
                    self._get_sensor(CH_LEFT_REAR))

    def _side_distance(self, side: str) -> float:
        """Average distance from the wall on *side*.
        Returns the wall-follow target if no data (→ zero correction).
        """
        front_val, rear_val = self._side_sensors(side)
        vals = [v for v in [front_val, rear_val] if v is not None]
        return sum(vals) / len(vals) if vals else self.row_wall_follow

    def _side_heading_error(self, side: str) -> float:
        """Front-side minus rear-side sensor on *side*.
        Positive = front of robot is farther from that wall.
        """
        front_val, rear_val = self._side_sensors(side)
        if front_val is None or rear_val is None:
            return 0.0
        return front_val - rear_val

    # ══════════════════════════════════════════════════════════════════════
    #  Motor Control
    # ══════════════════════════════════════════════════════════════════════

    def _send_motors(self, left_duty: int, right_duty: int):
        """Send duty to both RoboClaws.
        Positive = forward, negative = backward.
        """
        if self.controller is None:
            return

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

    # ══════════════════════════════════════════════════════════════════════
    #  Wall-Following Helper
    # ══════════════════════════════════════════════════════════════════════

    def _wall_follow_steer(self, side: str, dt: float) -> tuple:
        """Compute combined heading + lateral steering for wall following.

        Returns (steer, lateral_dist, heading_error, lateral_error) where:
            steer > 0 → steer right (left wheel faster)
            steer < 0 → steer left  (right wheel faster)
        """
        heading_error = self._side_heading_error(side)
        lateral_dist = self._side_distance(side)
        lateral_error = self.row_wall_follow - lateral_dist  # +ve = too close

        heading_corr = self.heading_pid.compute(heading_error, dt)
        lateral_corr = self.lateral_pid.compute(lateral_error, dt)

        # Sign convention:
        #   Following RIGHT wall:
        #     heading_error > 0 (nose far from right) → steer right (+)
        #     lateral_error > 0 (too close to right)  → steer left  (-)
        #   Following LEFT wall:
        #     heading_error > 0 (nose far from left)  → steer left  (-)
        #     lateral_error > 0 (too close to left)   → steer right (+)
        if side == 'right':
            steer = heading_corr - lateral_corr
        else:
            steer = -heading_corr + lateral_corr

        return steer, lateral_dist, heading_error, lateral_error

    # ══════════════════════════════════════════════════════════════════════
    #  Navigation Helpers
    # ══════════════════════════════════════════════════════════════════════

    def _reset_navigation(self):
        """Reset all navigation state for a fresh run."""
        self.row_count = 0
        self.u_turn_leg = 0
        self.follow_right_wall = True
        self.turn_direction = 1
        self.align_stable_since = None
        self.nav_start_back_distance = None
        self._log_counter = 0
        self.heading_pid.reset()
        self.lateral_pid.reset()
        self.front_pid.reset()

    def _start_turn(self):
        """Transition to EXECUTE_TURN."""
        self._stop_motors()
        self.phase_start_time = time.monotonic()
        self.state = State.EXECUTE_TURN
        direction_str = 'RIGHT' if self.turn_direction > 0 else 'LEFT'
        self.get_logger().info(f'>>> Starting turn {direction_str}')

    # ══════════════════════════════════════════════════════════════════════
    #  State Machine
    # ══════════════════════════════════════════════════════════════════════

    def _control_loop(self):
        """Main control loop — dispatches to the active state handler."""
        now = time.monotonic()
        dt = (now - self.last_control_time) if self.last_control_time else \
             (1.0 / self.control_rate)
        self.last_control_time = now

        if self.state == State.IDLE:
            pass
        elif self.state == State.NAVIGATE_TO_ROW:
            self._do_navigate_to_row(dt)
        elif self.state == State.EXECUTE_TURN:
            self._do_execute_turn(dt)
        elif self.state == State.ALIGN_TO_ROW:
            self._do_align_to_row(dt)
        elif self.state == State.DRIVE_ROW:
            self._do_drive_row(dt)
        elif self.state == State.APPROACH_WALL:
            self._do_approach_wall(dt)
        elif self.state == State.DRIVE_TO_NEXT_ROW:
            self._do_drive_to_next_row(dt)

    # ── NAVIGATE_TO_ROW ──────────────────────────────────────────────────

    def _do_navigate_to_row(self, dt: float):
        """Drive from corner to first row.  Follow LEFT wall (closest to
        the top-left starting corner).  Stop when front sensors detect
        the far wall / row entrance.
        """
        front_dist = self._front_distance()

        # Exit: close enough to far wall → time to turn into row 1
        if front_dist <= self.row_approach:
            self.get_logger().info(
                f'Row entrance reached (front={front_dist:.0f}mm) '
                f'— turning right into row 1'
            )
            self._stop_motors()
            self.row_count = 1
            self.follow_right_wall = True
            self.turn_direction = 1   # right turn to enter row 1
            self._start_turn()
            return

        # Wall-follow on LEFT side (left arena wall near starting corner)
        steer, lat_d, h_err, l_err = self._wall_follow_steer('left', dt)

        base = int(self.drive_duty_frac * DUTY_MAX)
        self._send_motors(base + int(steer), base - int(steer))

        # Log periodically
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'NAV_TO_ROW: left_wall={lat_d:.0f}mm '
                f'heading_err={h_err:.0f} front={front_dist:.0f}mm'
            )

    # ── EXECUTE_TURN ─────────────────────────────────────────────────────

    def _do_execute_turn(self, dt: float):
        """Open-loop timed spin burst.  Direction set by self.turn_direction."""
        elapsed = time.monotonic() - self.phase_start_time

        if elapsed >= self.turn_burst:
            self._stop_motors()
            # Reset PIDs for alignment phase
            self.heading_pid.reset()
            self.lateral_pid.reset()
            self.front_pid.reset()
            self.align_stable_since = None
            self.phase_start_time = time.monotonic()
            self.state = State.ALIGN_TO_ROW
            self.get_logger().info(
                f'Turn burst complete ({elapsed:.2f}s) — aligning'
            )
            return

        turn_duty = int(self.turn_duty_frac * DUTY_MAX)
        if self.turn_direction > 0:
            # Turn right: left forward, right backward
            self._send_motors(turn_duty, -turn_duty)
        else:
            # Turn left: left backward, right forward
            self._send_motors(-turn_duty, turn_duty)

    # ── ALIGN_TO_ROW ─────────────────────────────────────────────────────

    def _do_align_to_row(self, dt: float):
        """Square up using back sensors (CH4≈CH5) and check lateral position.

        After a turn, the back of the robot faces a wall.  PID-controlled
        small spin corrections drive CH4-CH5 toward zero (squared up).
        """
        # ── Squareness (heading) via back sensors ────────────────────────
        back_error = self._back_heading_error()
        # Positive back_error (left-back farther) → spin right to fix
        spin_correction = self.heading_pid.compute(back_error, dt)

        # Apply spin only (no forward drive during alignment)
        self._send_motors(int(spin_correction), -int(spin_correction))

        # ── Check tolerances ─────────────────────────────────────────────
        side = 'right' if self.follow_right_wall else 'left'
        lateral_dist = self._side_distance(side)
        lateral_error = abs(self.row_wall_follow - lateral_dist)

        heading_ok = abs(back_error) < self.align_heading_tol
        lateral_ok = lateral_error < self.align_lateral_tol

        all_ok = heading_ok and lateral_ok

        if all_ok:
            if self.align_stable_since is None:
                self.align_stable_since = time.monotonic()
            elif time.monotonic() - self.align_stable_since >= self.align_hold_time:
                # ── Aligned!  Determine next state ───────────────────────
                self._stop_motors()
                self.heading_pid.reset()
                self.lateral_pid.reset()
                self.front_pid.reset()

                if self.u_turn_leg == 1:
                    # First turn of U-turn done → drive to next row
                    self.nav_start_back_distance = self._back_distance()
                    self.u_turn_leg = 2
                    self.phase_start_time = time.monotonic()
                    self.state = State.DRIVE_TO_NEXT_ROW
                    self.get_logger().info(
                        'Aligned — driving to next row '
                        f'(back_start={self.nav_start_back_distance:.0f}mm)'
                    )
                elif self.u_turn_leg == 2:
                    # Second turn of U-turn done → enter new row
                    self.u_turn_leg = 0
                    self.phase_start_time = time.monotonic()
                    self.state = State.DRIVE_ROW
                    side_str = 'RIGHT' if self.follow_right_wall else 'LEFT'
                    self.get_logger().info(
                        f'Aligned — entering row {self.row_count} '
                        f'(follow {side_str} wall)'
                    )
                else:
                    # Initial turn into first row
                    self.u_turn_leg = 0
                    self.phase_start_time = time.monotonic()
                    self.state = State.DRIVE_ROW
                    side_str = 'RIGHT' if self.follow_right_wall else 'LEFT'
                    self.get_logger().info(
                        f'Aligned — entering row {self.row_count} '
                        f'(follow {side_str} wall)'
                    )
        else:
            self.align_stable_since = None

        # Log periodically
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'ALIGN: back_err={back_error:.0f}mm '
                f'lateral={lateral_dist:.0f}mm '
                f'ok=({heading_ok},{lateral_ok}) '
                f'u_leg={self.u_turn_leg}'
            )

    # ── DRIVE_ROW ────────────────────────────────────────────────────────

    def _do_drive_row(self, dt: float):
        """PID wall-following through a corn row."""
        front_dist = self._front_distance()

        # Exit: approaching end-of-row wall
        if front_dist <= self.row_approach:
            self.get_logger().info(
                f'End of row {self.row_count} '
                f'(front={front_dist:.0f}mm) — approaching wall'
            )
            self.heading_pid.reset()
            self.lateral_pid.reset()
            self.front_pid.reset()
            self.phase_start_time = time.monotonic()
            self.state = State.APPROACH_WALL
            return

        # Wall follow on active side
        side = 'right' if self.follow_right_wall else 'left'
        steer, lat_d, h_err, l_err = self._wall_follow_steer(side, dt)

        base = int(self.drive_duty_frac * DUTY_MAX)
        self._send_motors(base + int(steer), base - int(steer))

        # Log periodically
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'DRIVE_ROW({self.row_count}): '
                f'{side}_wall={lat_d:.0f}mm h_err={h_err:.0f} '
                f'front={front_dist:.0f}mm'
            )

    # ── APPROACH_WALL ────────────────────────────────────────────────────

    def _do_approach_wall(self, dt: float):
        """PID-controlled deceleration at end of row."""
        front_dist = self._front_distance()

        # Exit: close enough to wall → start U-turn
        if front_dist <= self.approach_stop:
            self.get_logger().info(
                f'At wall (front={front_dist:.0f}mm) — starting U-turn '
                f'for row {self.row_count}'
            )
            self._stop_motors()

            # Set up U-turn
            self.u_turn_leg = 1
            if self.row_count % 2 == 1:   # odd row → turn right
                self.turn_direction = 1
            else:                          # even row → turn left
                self.turn_direction = -1
            self._start_turn()
            return

        # Front-distance PID → forward speed (slows as we approach)
        front_error = front_dist - self.approach_stop   # positive = still far
        forward_speed = self.front_pid.compute(front_error, dt)

        # Maintain lateral position while decelerating
        side = 'right' if self.follow_right_wall else 'left'
        steer, _, _, _ = self._wall_follow_steer(side, dt)

        self._send_motors(int(forward_speed) + int(steer),
                          int(forward_speed) - int(steer))

        # Log periodically
        self._log_counter += 1
        if self._log_counter % 20 == 0:
            self.get_logger().info(
                f'APPROACH: front={front_dist:.0f}mm '
                f'fwd_speed={forward_speed:.0f}'
            )

    # ── DRIVE_TO_NEXT_ROW ────────────────────────────────────────────────

    def _do_drive_to_next_row(self, dt: float):
        """Drive between U-turn legs.  Track distance using back sensors.
        Drive straight (no wall-following) since the distance is short (~178mm).
        """
        current_back = self._back_distance()
        distance_traveled = abs(current_back - self.nav_start_back_distance) \
            if self.nav_start_back_distance is not None else 0.0

        # Exit: reached next row position
        if distance_traveled >= self.next_row_spacing:
            self.get_logger().info(
                f'At next row (traveled {distance_traveled:.0f}mm) '
                f'— turning into row {self.row_count + 1}'
            )
            self._stop_motors()

            # Set up second turn of U-turn (same direction as first)
            self.row_count += 1
            self.follow_right_wall = (self.row_count % 2 == 1)
            # turn_direction stays the same as the first U-turn leg
            self._start_turn()
            return

        # Drive straight at base speed
        base = int(self.drive_duty_frac * DUTY_MAX)
        self._send_motors(base, base)

        # Log periodically
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'DRIVE_NEXT_ROW: traveled={distance_traveled:.0f}mm '
                f'/ {self.next_row_spacing:.0f}mm '
                f'(back={current_back:.0f}mm)'
            )

    # ══════════════════════════════════════════════════════════════════════
    #  Cleanup
    # ══════════════════════════════════════════════════════════════════════

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

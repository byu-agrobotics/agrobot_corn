#!/usr/bin/env python3
"""
PID Navigation State Machine
=============================

Subscribes to 8 ToF sensor topics, implements a state machine for
autonomous navigation with PID wall-following, and controls the
RoboClaws directly.

Sensor layout (looking down at the robot):

        FRONT
   ┌──────────────┐
   │ CH0      CH1 │   ← front sensors (end-wall detection)
   │              │
   │CH2        CH3│   ← side sensors (wall-following)
   │              │
   │CH4        CH5│   ← side sensors (wall-following)
   │              │
   │ CH6      CH7 │   ← rear sensors
   └──────────────┘
        REAR

Left wall following uses CH2 + CH4 (averaged).
Front wall detection uses CH0 + CH1 (minimum of the two).

States:
    IDLE           → motors stopped, waiting for start command
    DRIVE_STRAIGHT → PID wall-following on left side
    STOPPING       → front wall detected, decelerating to stop
    TURNING_RIGHT  → slow 90° right turn using ToF feedback
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
CH_RIGHT_FRONT = 3   # right side, toward the front of the robot
CH_LEFT_REAR   = 4   # left side, toward the rear of the robot
CH_RIGHT_REAR  = 5   # right side, toward the rear of the robot
CH_REAR_LEFT   = 6
CH_REAR_RIGHT  = 7


class State(Enum):
    IDLE = auto()
    DRIVE_STRAIGHT = auto()
    STOPPING = auto()
    TURNING_RIGHT = auto()


class NavStateMachine(Node):
    """PID navigation state machine with ToF wall-following."""

    def __init__(self):
        super().__init__('nav_state_machine')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('wall_distance_mm', 65.0)
        self.declare_parameter('front_stop_distance_mm', 110.0)
        self.declare_parameter('collision_distance_mm', 35.0)
        self.declare_parameter('drive_duty_fraction', 0.35)
        self.declare_parameter('turn_duty_fraction', 0.30)
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.005)
        self.declare_parameter('kd', 0.5)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('turn_clear_distance_mm', 180.0)

        self.wall_setpoint = self.get_parameter('wall_distance_mm').value
        self.front_stop_dist = self.get_parameter('front_stop_distance_mm').value
        self.collision_dist = self.get_parameter('collision_distance_mm').value
        self.drive_duty_frac = self.get_parameter('drive_duty_fraction').value
        self.turn_duty_frac = self.get_parameter('turn_duty_fraction').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.turn_clear_dist = self.get_parameter('turn_clear_distance_mm').value


        # ── State ────────────────────────────────────────────────────────────
        self.state = State.IDLE
        self.distances = [0.0] * NUM_SENSORS   # latest reading per channel
        self.sensor_valid = [False] * NUM_SENSORS  # have we received at least one msg?

        # PID state
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_time = None

        # Turn state
        self.turn_start_time = None
        self.turn_max_duration = 8.0  # safety timeout (seconds)
        self.turn_phase = 0  # 0 = clearing front, 1 = waiting for alignment

        # Collision detection: require consecutive readings before triggering
        self.collision_counts = [0] * NUM_SENSORS
        self.collision_threshold_count = 5  # consecutive readings needed (~250ms at 20Hz)

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
            f'Front stop: {self.front_stop_dist}mm, '
            f'PID: Kp={self.kp} Ki={self.ki} Kd={self.kd}'
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
                self.state = State.DRIVE_STRAIGHT
        elif msg.num == 0:  # 'q' key
            if self.state != State.IDLE:
                self.get_logger().info('>>> STOP: → IDLE')
                self._stop_motors()
                self.state = State.IDLE

    # ── Sensor helpers ───────────────────────────────────────────────────────

    def _front_distance(self) -> float:
        """Minimum of the two front sensors (conservative for safety)."""
        vals = []
        if self.sensor_valid[CH_FRONT_LEFT] and self.distances[CH_FRONT_LEFT] > 0:
            vals.append(self.distances[CH_FRONT_LEFT])
        if self.sensor_valid[CH_FRONT_RIGHT] and self.distances[CH_FRONT_RIGHT] > 0:
            vals.append(self.distances[CH_FRONT_RIGHT])
        if not vals:
            return 9999.0  # no valid reading → assume clear
        return min(vals)

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

    def _any_collision_risk(self) -> bool:
        """Check if ANY sensor has consecutive readings below collision threshold."""
        collision_detected = False
        for ch in range(NUM_SENSORS):
            if self.sensor_valid[ch] and 0 < self.distances[ch] < self.collision_dist:
                self.collision_counts[ch] += 1
                if self.collision_counts[ch] >= self.collision_threshold_count:
                    self.get_logger().warn(
                        f'COLLISION RISK: CH{ch} = {self.distances[ch]:.0f}mm '
                        f'for {self.collision_counts[ch]} consecutive readings'
                    )
                    collision_detected = True
            else:
                self.collision_counts[ch] = 0  # reset if reading is normal
        return collision_detected

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
                self.controller.DutyM1M2(self.addr_1, left_duty, right_duty)
            except Exception as e:
                self.get_logger().error(f'RoboClaw 1 error: {e}')
            try:
                self.controller.DutyM1M2(self.addr_2, left_duty, right_duty)
            except Exception as e:
                self.get_logger().error(f'RoboClaw 2 error: {e}')

    def _stop_motors(self):
        """Immediately stop all motors."""
        self._send_motors(0, 0)

    # ── State machine ────────────────────────────────────────────────────────

    def _control_loop(self):
        """Main control loop — runs at control_rate_hz."""

        # ── Safety first: collision avoidance in ALL states ──────────────────
        if self.state != State.IDLE and self._any_collision_risk():
            self.get_logger().error('EMERGENCY STOP — collision risk detected!')
            self._stop_motors()
            self.state = State.IDLE
            return

        # ── State dispatch ───────────────────────────────────────────────────
        if self.state == State.IDLE:
            pass  # do nothing, motors should already be stopped

        elif self.state == State.DRIVE_STRAIGHT:
            self._do_drive_straight()

        elif self.state == State.STOPPING:
            self._do_stopping()

        elif self.state == State.TURNING_RIGHT:
            self._do_turning_right()

    def _do_drive_straight(self):
        """PID wall-following: maintain fixed distance from left wall."""

        # Check front wall
        front_dist = self._front_distance()
        if front_dist < self.front_stop_dist:
            self.get_logger().info(
                f'Front wall detected at {front_dist:.0f}mm — STOPPING'
            )
            self._stop_motors()
            self.state = State.STOPPING
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
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            self.get_logger().info(
                f'DRIVE: left_wall={left_dist:.0f}mm '
                f'front={front_dist:.0f}mm '
                f'correction={correction:.0f} '
                f'duty=({left_duty},{right_duty})'
            )

    def _do_stopping(self):
        """Stopped at wall — transition to turn."""
        self._stop_motors()

        self.get_logger().info('Stopped at wall — starting RIGHT TURN')
        self.turn_start_time = time.monotonic()
        self.turn_phase = 0
        self.state = State.TURNING_RIGHT

    def _do_turning_right(self):
        """Execute a slow 90° right turn using ToF feedback."""
        elapsed = time.monotonic() - self.turn_start_time

        # Safety timeout
        if elapsed > self.turn_max_duration:
            self.get_logger().warn(
                f'Turn timeout after {self.turn_max_duration}s — stopping'
            )
            self._stop_motors()
            self.state = State.IDLE
            return

        front_dist = self._front_distance()
        left_dist = self._left_wall_distance()
        turn_duty = int(self.turn_duty_frac * DUTY_MAX)

        if self.turn_phase == 0:
            # Phase 0: Spin right until front sensors clear the wall
            # (front distance exceeds the "clear" threshold)
            self._send_motors(turn_duty, -turn_duty)

            if front_dist > self.turn_clear_dist:
                self.get_logger().info(
                    f'Turn phase 0 complete: front clear at {front_dist:.0f}mm '
                    f'(elapsed {elapsed:.1f}s)'
                )
                self.turn_phase = 1

        elif self.turn_phase == 1:
            # Phase 1: Continue spinning right until the left sensors
            # detect a wall at approximately the wall-following setpoint.
            # This means the robot has rotated ~90° and is now aligned
            # with a wall on its left side.
            self._send_motors(turn_duty, -turn_duty)

            # Check if left sensors see a wall near the setpoint
            # (within a tolerance band)
            tolerance = 40.0  # mm
            if (self.sensor_valid[CH_LEFT_FRONT] or self.sensor_valid[CH_LEFT_REAR]):
                if abs(left_dist - self.wall_setpoint) < tolerance and left_dist > 0:
                    self.get_logger().info(
                        f'Turn phase 1 complete: left wall at {left_dist:.0f}mm '
                        f'(target {self.wall_setpoint:.0f}mm, elapsed {elapsed:.1f}s)'
                    )
                    self._stop_motors()
                    self._reset_pid()
                    self.state = State.DRIVE_STRAIGHT
                    self.get_logger().info('Turn complete — resuming DRIVE_STRAIGHT')
                    return

            # Also check: if front sees a wall close again (turned too far),
            # stop the turn immediately
            if front_dist < self.front_stop_dist and elapsed > 1.0:
                self.get_logger().warn(
                    f'Front wall reappeared at {front_dist:.0f}mm during turn — stopping'
                )
                self._stop_motors()
                self.state = State.IDLE

        # Log turn progress periodically
        if not hasattr(self, '_turn_log_counter'):
            self._turn_log_counter = 0
        self._turn_log_counter += 1
        if self._turn_log_counter % 10 == 0:
            self.get_logger().info(
                f'TURNING: phase={self.turn_phase} '
                f'front={front_dist:.0f}mm left={left_dist:.0f}mm '
                f'elapsed={elapsed:.1f}s'
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

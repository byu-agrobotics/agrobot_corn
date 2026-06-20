import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tutorial_interfaces.msg import Num

import sys
import os
import threading

# Ensure the venv site-packages are on sys.path so basicmicro can be found
# even when ros2 run uses /usr/bin/python3
_venv_site = os.path.expanduser('~/ros2_ws/venv/lib/python3.12/site-packages')
if _venv_site not in sys.path:
    sys.path.insert(0, _venv_site)

try:
    from basicmicro import Basicmicro
except Exception:
    Basicmicro = None


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.controller = None
        self.controller_lock = threading.Lock()
        self.controller_address_1 = 0x80  # First RoboClaw (128)
        self.controller_address_2 = 0x81  # Second RoboClaw (129) - mirrors first

        # Track consecutive failures per controller so we can stop retrying a dead one
        self.fail_count_1 = 0
        self.fail_count_2 = 0
        self.max_failures = 3  # disable a controller after this many consecutive failures

        # Try to connect to the Basicmicro (RoboClaw) controller
        if Basicmicro is not None:
            # Try common serial ports for RoboClaw on Raspberry Pi
            ports_to_try = ['/dev/ttyAMA0', '/dev/ttyACM0', '/dev/ttyUSB0']
            for port in ports_to_try:
                if not os.path.exists(port):
                    continue
                try:
                    self.controller = Basicmicro(port, 38400)
                    if self.controller.Open():
                        self.get_logger().info(f'Connected to Basicmicro controller on {port}')
                        break
                    else:
                        self.get_logger().warn(f'Basicmicro failed to open on {port}')
                        self.controller = None
                except Exception as e:
                    self.get_logger().warn(f'Basicmicro init failed on {port}: {e}')
                    self.controller = None

            if self.controller is None:
                self.get_logger().error('Could not connect to Basicmicro on any port!')
        else:
            self.get_logger().error('basicmicro library not available! Install with: pip install basicmicro')

    def _send_duty(self, address, left, right, label):
        """Send DutyM1M2 to one controller. Returns True on success."""
        try:
            ok = self.controller.DutyM1M2(address, left, right)
            if not ok:
                self.get_logger().warn(f'DutyM1M2 returned failure on {label}')
                return False
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending duty to {label}: {e}')
            return False

    def listener_callback(self, msg):
        # Map command numbers to duty values for Basicmicro (range -32767..32767)
        duty_full = 32767
        try:
            if msg.num == 1:
                # Straight forward
                self.get_logger().info('Going straight')
                left = int(0.7 * duty_full)
                right = int(0.7 * duty_full)
            elif msg.num == 2:
                # Turn left (left backward, right forward)
                self.get_logger().info('Turning left')
                left = -int(0.7 * duty_full)
                right = int(0.7 * duty_full)
            elif msg.num == 3:
                # Reverse
                self.get_logger().info('Reversing')
                left = -int(0.7 * duty_full)
                right = -int(0.7 * duty_full)
            elif msg.num == 4:
                # Turn right (left forward, right backward)
                self.get_logger().info('Turning right')
                left = int(0.7 * duty_full)
                right = -int(0.7 * duty_full)
            elif msg.num == 0:
                # Stop
                self.get_logger().info('Stopping')
                left = 0
                right = 0
            else:
                # Unknown command: ignore
                return

            if self.controller is None:
                self.get_logger().warn('No controller connected - motors not moving')
                return

            with self.controller_lock:
                # Send to RoboClaw 1 (independent try so failure doesn't block RoboClaw 2)
                if self.fail_count_1 < self.max_failures:
                    if self._send_duty(self.controller_address_1, left, right, 'RoboClaw 1 (0x80)'):
                        self.fail_count_1 = 0
                    else:
                        self.fail_count_1 += 1
                        if self.fail_count_1 >= self.max_failures:
                            self.get_logger().error('RoboClaw 1 (0x80) disabled after repeated failures')

                # Send to RoboClaw 2 (independent try so failure doesn't block RoboClaw 1)
                if self.fail_count_2 < self.max_failures:
                    if self._send_duty(self.controller_address_2, left, right, 'RoboClaw 2 (0x81)'):
                        self.fail_count_2 = 0
                    else:
                        self.fail_count_2 += 1
                        if self.fail_count_2 >= self.max_failures:
                            self.get_logger().error('RoboClaw 2 (0x81) disabled after repeated failures')

        except Exception as e:
            self.get_logger().error(f'Unhandled exception in listener_callback: {e}')


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_subscriber = MinimalSubscriber()

            rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
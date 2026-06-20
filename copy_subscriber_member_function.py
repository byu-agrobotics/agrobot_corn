import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tutorial_interfaces.msg import Num

import threading
try:
    from basicmicro import Basicmicro
except Exception:
    Basicmicro = None

try:
    from gpiozero import PWMOutputDevice, OutputDevice
except Exception:
    PWMOutputDevice = None
    OutputDevice = None


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
        self.controller_address = 128

        # Try to use Basicmicro controller if available
        if Basicmicro is not None:
            try:
                self.controller = Basicmicro('/dev/ttyACM0', 38400)
                if self.controller.Open():
                    self.get_logger().info('Connected to Basicmicro controller')
                else:
                    self.get_logger().warn('Basicmicro library present but failed to open serial port')
                    self.controller = None
            except Exception as e:
                self.get_logger().error(f'Basicmicro init error: {e}')
                self.controller = None

        # If Basicmicro not available, fallback to gpiozero PWM pins (for testing without RoboClaw)
        if self.controller is None and PWMOutputDevice is not None and OutputDevice is not None:
            self.get_logger().info('Falling back to gpiozero PWM outputs')
            self.motor_speed_pin_right = PWMOutputDevice(12)
            self.motor_speed_pin_left = PWMOutputDevice(13)
            self.signal_pin_23 = OutputDevice(23)
            self.signal_pin_22 = OutputDevice(22)
            self.signal_pin_27 = OutputDevice(27)
            self.signal_pin_17 = OutputDevice(17)
            self.signal_pin_23.on()
            self.signal_pin_22.off()
            self.signal_pin_27.on()
            self.signal_pin_17.off()
        else:
            # If using controller, ensure the GPIO signal pins are not used
            self.motor_speed_pin_right = None
            self.motor_speed_pin_left = None
            self.signal_pin_23 = None
            self.signal_pin_22 = None
            self.signal_pin_27 = None
            self.signal_pin_17 = None

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

            # If controller available, send DutyM1M2
            if self.controller is not None:
                with self.controller_lock:
                    try:
                        # controller.DutyM1M2(address, left, right)
                        success = self.controller.DutyM1M2(self.controller_address, left, right)
                        if not success:
                            self.get_logger().warn('DutyM1M2 command failed')
                    except Exception as e:
                        self.get_logger().error(f'Error sending duty command: {e}')
            else:
                # Fallback to gpiozero PWM outputs
                if self.motor_speed_pin_right is not None and self.motor_speed_pin_left is not None:
                    # pwm.value expects -1..1? gpiozero PWMOutputDevice.value is 0..1, so map accordingly
                    # Here we use absolute value and direction pins to emulate previous behaviour
                    # Keep simple mapping: set value to abs(left)/duty_full for left and abs(right)/duty_full for right
                    try:
                        self.motor_speed_pin_left.value = min(1.0, abs(left) / duty_full)
                        self.motor_speed_pin_right.value = min(1.0, abs(right) / duty_full)
                    except Exception as e:
                        self.get_logger().error(f'gpiozero PWM error: {e}')

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
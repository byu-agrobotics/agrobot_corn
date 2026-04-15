from gpiozero import PWMOutputDevice, OutputDevice
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from tutorial_interfaces.msg import Num                      # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                               # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
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

    def listener_callback(self, msg):
        if (msg.num == 1):
            # Straight
            self.signal_pin_23.off()
            self.signal_pin_22.on()
            self.signal_pin_27.off()
            self.signal_pin_17.on()
            self.get_logger().info('Going straight at 0.5')     
            self.motor_speed_pin_right.value = 0.7
            self.motor_speed_pin_left.value = 0.7
        if (msg.num == 2):
            # Turn Left
            self.signal_pin_23.on()
            self.signal_pin_22.off()
            self.signal_pin_27.off()
            self.signal_pin_17.on()
            self.get_logger().info('Turning left at 0.5')
            self.motor_speed_pin_right.value = 1
            self.motor_speed_pin_left.value = 1
        if (msg.num == 3):
            # Reverse
            self.signal_pin_23.on()
            self.signal_pin_22.off()
            self.signal_pin_27.on()
            self.signal_pin_17.off()
            self.get_logger().info('Reversing at 0.5')
            self.motor_speed_pin_right.value = 0.7
            self.motor_speed_pin_left.value = 0.7
        if (msg.num == 4):
            # Turn Right
            self.signal_pin_23.off()
            self.signal_pin_22.on()
            self.signal_pin_27.on()
            self.signal_pin_17.off()
            self.get_logger().info('Turning right at 0.5')
            self.motor_speed_pin_right.value = 1
            self.motor_speed_pin_left.value = 1
        if (msg.num == 0):
            # Stop
            self.signal_pin_23.off()
            self.signal_pin_22.off()
            self.signal_pin_27.off()
            self.signal_pin_17.off()
            self.get_logger().info('Stopping')
            self.motor_speed_pin_right.value = 0.0
            self.motor_speed_pin_left.value = 0.0


def main(args=None):
    try:
        with rclpy.init(args=args):
            minimal_subscriber = MinimalSubscriber()

            rclpy.spin(minimal_subscriber)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
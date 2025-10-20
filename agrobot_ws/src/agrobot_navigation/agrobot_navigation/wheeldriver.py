#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from agrobot_interfaces.msg import DriveCommand

'''
Ros2 node that directly controls treads in two-wheel drive
subscribes: drive_command (agrobot_interfaces/DriveCommand.msg)
 - for my implementation, ranges are from -100 to 100

Note: no pid controller implemented right now. we will want pid implemented here since it should run quickly.
'''
class WheelDriver(Node):
    def __init__(self):
        super().__init__('my_node')  # Node name
        self.get_logger().info("Node has been started!")

        # GPIO pin setup (Hardcoded: expose as ros params?)
        self.right_pwm_pin = 18
        self.right_dir_pin = 23
        self.left_pwm_pin = 24
        self.left_dir_pin = 25

        # configures gpio pins connect to wheels
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.right_pwm_pin, GPIO.OUT)
        GPIO.setup(self.right_dir_pin, GPIO.OUT)
        GPIO.setup(self.left_pwm_pin, GPIO.OUT)
        GPIO.setup(self.left_dir_pin, GPIO.OUT)

        self.right_pwm = GPIO.PWM(self.right_pwm_pin, 1000)
        self.left_pwm = GPIO.PWM(self.left_pwm_pin, 1000)
        self.right_pwm.start(0)
        self.left_pwm.start(0)

        self.drive_sub = self.create_subscription(
            DriveCommand,
            'drive_command',
            self.reset_drive_callback,
            10
        )

    def set_motor(self, pwm, dir_pin, speed):
        speed = max(-100, min(100, speed))
        if speed >= 0:
            GPIO.output(dir_pin, GPIO.HIGH)
            pwm.ChangeDutyCycle(speed)
        else:
            GPIO.output(dir_pin, GPIO.LOW)
            pwm.ChangeDutyCycle(-speed)

    def reset_drive_callback(self, msg:DriveCommand):
        self.get_logger().info(f'Received DriveCommand: R={msg.right_speed}, L={msg.left_speed}')
        self.set_motor(self.right_pwm, self.right_dir_pin, msg.right_speed)
        self.set_motor(self.left_pwm, self.left_dir_pin, msg.left_speed)

    def destroy_node(self):
        self.right_pwm.stop()
        self.left_pwm.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    wheel_driver = WheelDriver()
    try:
        rclpy.spin(wheel_driver)
    except KeyboardInterrupt:
        pass
    finally:
        wheel_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

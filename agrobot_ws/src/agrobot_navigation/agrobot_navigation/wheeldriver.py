#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from agrobot_interfaces.msg import DriveCommand

'''
Ros2 node that directly controls treads in two-wheel drive
subscribes: drive_command (agrobot_interfaces/DriveCommand.msg)
 - for my implementation, ranges are from -10 to 10

Note: no pid controller implemented right now. we will want pid implemented here since it should run quickly.
'''
class WheelDriver(Node):
    def __init__(self):
        super().__init__('my_node')  # Node name
        self.get_logger().info("Node has been started!")

        # GPIO pin setup (Hardcoded: expose as ros params?)
        self.right_forward_pin = 7
        self.right_backward_pin = 1
        self.left_forward_pin = 25
        self.left_backward_pin = 8

        # configures gpio pins connect to wheels
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.right_pwm_pin, GPIO.OUT)
        GPIO.setup(self.right_dir_pin, GPIO.OUT)
        GPIO.setup(self.left_pwm_pin, GPIO.OUT)
        GPIO.setup(self.left_dir_pin, GPIO.OUT)

        # self.right_pwm = GPIO.PWM(self.right_pwm_pin, 1000)
        # self.left_pwm = GPIO.PWM(self.left_pwm_pin, 1000)
        # self.right_pwm.start(0)
        # self.left_pwm.start(0)

        self.drive_sub = self.create_subscription(
            DriveCommand,
            'drive_command',
            self.reset_drive_callback,
            10
        )

        # timer_period = 0.01  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.left_speed = 0
        # self.right_speed = 0
        # self.i=0
        # self.pwm_period = 10


    # def software_pwm(self, pin, speed):
    #     if(self.i<speed):
    #         GPIO.output(pin, GPIO.HIGH)
    #     else:
    #         GPIO.output(pin, GPIO.LOW)

    # def timer_callback(self):
    #     self.software_pwm(self.left_pwm_pin, self.left_speed)
    #     self.software_pwm(self.right_pwm_pin, self.right_speed)
    #     self.i += 1
    #     if(self.i>=10): self.i=0

    def set_motor(self, forward_pin, backward_pin, speed):
        speed = max(-100, min(100, speed))
        if speed > 0:
            GPIO.output(forward_pin, GPIO.HIGH)
            GPIO.output(backward_pin, GPIO.LOW)
            # self.left_speed = speed
        elif speed < 0:
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.HIGH)
        else:
            GPIO.output(forward_pin, GPIO.LOW)
            GPIO.output(backward_pin, GPIO.LOW)
            # self.right_speed = speed

    def reset_drive_callback(self, msg:DriveCommand):
        self.get_logger().info(f'Received DriveCommand: R={msg.right_speed}, L={msg.left_speed}')
        self.set_motor(self.right_dir_pin, msg.right_speed)
        self.set_motor(self.left_dir_pin, msg.left_speed)

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
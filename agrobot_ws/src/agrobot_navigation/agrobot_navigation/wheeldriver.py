#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# import RPi.GPIO as GPIO
from gpiozero import OutputDevice, PWMOutputDevice
from agrobot_interfaces.msg import DriveCommand


class Wheel:
    def __init__(self, 
                 forward_pin_num:int,
                 backward_pin_num:int,
                 pwm_pin_num:int):
        self.forward_pin_num = forward_pin_num
        self.backward_pin_num = backward_pin_num
        self.pwm_pin_num = pwm_pin_num
        self.forward_pin = OutputDevice(forward_pin_num)
        self.backward_pin = OutputDevice(backward_pin_num)
        self.pwm = PWMOutputDevice(pwm_pin_num)

    def set_motor(self, speed):
        speed = max(-100, min(100, speed))
        self.pwm.value = abs(speed)/100
        if speed > 0:
            self.forward_pin.on()
            self.backward_pin.off()
        elif speed < 0:
            self.forward_pin.off()
            self.backward_pin.on()
        else:
            self.forward_pin.off()
            self.backward_pin.off()

    def close(self):
        self.__del__()

    def __del__(self):
        self.backward_pin.close()
        self.forward_pin.close()
        self.pwm.close()


'''
Ros2 node that directly controls treads in two-wheel drive
subscribes: drive_command (agrobot_interfaces/DriveCommand.msg)
 - for my implementation, ranges are from -10 to 10
'''
class WheelDriver(Node):
    def __init__(self):
        super().__init__('wheeldriver')  # Node name
        self.get_logger().info("Node has been started!")

        # GPIO pin setup (Hardcoded: expose as ros params?)
        right_forward_pin = 25
        right_backward_pin = 8
        right_pwm_pin = 23
        left_forward_pin = 7
        left_backward_pin = 1
        left_pwm_pin = 24

        # configures gpio pins connect to wheels
        self.right_wheel = Wheel(
            right_forward_pin, right_backward_pin, right_pwm_pin)
        self.left_wheel = Wheel(
            left_forward_pin, left_backward_pin, left_pwm_pin)

        self.drive_sub = self.create_subscription(
            DriveCommand,
            'drive_command',
            self.reset_drive_callback,
            10
        )

    def reset_drive_callback(self, msg:DriveCommand):
        self.get_logger().info(f'Received DriveCommand: R={msg.right_speed}, L={msg.left_speed}')
        self.left_wheel.set_motor(msg.left_speed)
        self.right_wheel.set_motor(msg.right_speed)

    def destroy_node(self):
        self.right_wheel.close()
        self.left_wheel.close()
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
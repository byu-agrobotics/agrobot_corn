#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from agrobot_interfaces.msg import DriveCommand

'''
Ros2 node that directly controls treads in two-wheel drive
subscribes: drive_command (agrobot_interfaces/DriveCommand.msg)

Note: no pid controller implemented right now. we will want pid implemented here since it should run quickly.
'''
class WheelsNode(Node):
    def __init__(self):
        super().__init__('my_node')  # Node name
        self.get_logger().info("Node has been started!")

        # Example timer: calls self.timer_callback every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Example publisher (commented out)
        # from std_msgs.msg import String
        # self.publisher_ = self.create_publisher(String, 'topic_name', 10)

        # Example subscriber (commented out)
        # self.subscription = self.create_subscription(
        #     String,
        #     'topic_name',
        #     self.listener_callback,
        #     10
        # )

        self.drive_sub = self.create_subscription(
            DriveCommand,
            'drive_command',
            self.reset_drive_callback,
            10
        )

    def reset_drive_callback(self, drivecommand:DriveCommand):
        pass

    # def timer_callback(self):
    #     # Example repeating callback
    #     self.get_logger().info("Timer callback running...")

    # Example subscription callback
    # def listener_callback(self, msg):
    #     self.get_logger().info(f"Received: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

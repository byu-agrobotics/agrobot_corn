#!/usr/bin/env python3
"""
LCD Display Publisher Node

Simulates another program by periodically publishing a text message and
a background color to topics. Used for testing the LCD subscriber.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA


class LCDDisplayPublisher(Node):
    """Publishes simulated display data: text and background color."""

    def __init__(self):
        super().__init__('lcd_display_publisher')
        self.text_pub = self.create_publisher(String, 'lcd_display/text', 10)
        self.color_pub = self.create_publisher(ColorRGBA, 'lcd_display/color', 10)
        self.timer = self.create_timer(3.0, self.timer_callback)
        self._cycle = 0

        # Simulated messages to cycle through
        self._messages = [
            'Hello Agrobot!',
            'System Ready',
            'Waiting for input...',
            'Processing...',
            'Task Complete',
        ]
        self._colors = [
            (0.1, 0.1, 0.44, 1.0),   # Navy blue
            (0.0, 0.39, 0.0, 1.0),   # Dark green
            (0.55, 0.0, 0.0, 1.0),   # Dark red
            (0.2, 0.2, 0.2, 1.0),    # Dark gray
            (0.44, 0.16, 0.16, 1.0), # Brown
        ]

    def timer_callback(self):
        """Publish next message and color in the cycle."""
        idx = self._cycle % len(self._messages)
        text_msg = String()
        text_msg.data = self._messages[idx]
        self.text_pub.publish(text_msg)

        color_msg = ColorRGBA()
        r, g, b, a = self._colors[idx]
        color_msg.r = r
        color_msg.g = g
        color_msg.b = b
        color_msg.a = a
        self.color_pub.publish(color_msg)

        self.get_logger().info(f'Published: "{text_msg.data}" with color ({r:.2f}, {g:.2f}, {b:.2f})')
        self._cycle += 1


def main(args=None):
    rclpy.init(args=args)
    node = LCDDisplayPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
Vision LCD Publisher Node

Runs the plant detection pipeline from VisionApplication and publishes
LCD display updates when plants are counted. Replaces the fake LCD publisher.

Color mapping (per ASABE rules):
  - Single Plant (green_1_stalk) -> Green
  - Double Plant (double_stalk) -> Blue
  - Empty Plant (base) -> Red

Text format: S: <single>, D: <double>, E: <empty>
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA

from agrobot_detect.vision_module import VisionApplication

# LCD color mapping (r, g, b, a) in 0-1 range
LCD_COLORS = {
    'green_1_stalk': (0.0, 0.8, 0.0, 1.0),   # Green - Single Plant
    'double_stalk': (0.0, 0.0, 0.8, 1.0),    # Blue - Double Plant
    'base': (0.8, 0.0, 0.0, 1.0),            # Red - Empty Plant
}
DEFAULT_COLOR = (0.1, 0.1, 0.44, 1.0)  # Navy when no counts yet


def _format_counts(total_count: dict) -> str:
    """Format counts as S: x, D: y, E: z."""
    s = total_count.get('green_1_stalk', 0)
    d = total_count.get('double_stalk', 0)
    e = total_count.get('base', 0)
    return f'S: {s}, D: {d}, E: {e}'


class VisionLCDPublisher(Node):
    """Runs vision pipeline and publishes LCD text/color on count events."""

    def __init__(self):
        super().__init__('vision_lcd_publisher')
        self.text_pub = self.create_publisher(String, 'lcd_display/text', 10)
        self.color_pub = self.create_publisher(ColorRGBA, 'lcd_display/color', 10)

        use_pi_camera = True
        try:
            from picamera2 import Picamera2  # noqa: F401
        except ImportError:
            use_pi_camera = False
            self.get_logger().warn('picamera2 not found; using OpenCV/V4L2 camera.')

        self._vision_app = VisionApplication(
            use_pi_camera=use_pi_camera,
            camera_index=0,
            camera2_index=1,
            camera_width=640,
            camera_height=480,
            extra_tolerance_factor=0.20,
            on_count_callback=self._on_count,
            streaming_video=False,
            use_gui=False,
        )

        # Publish initial state
        self._publish_text(_format_counts({}))
        self._publish_color(DEFAULT_COLOR)

        self._vision_thread = threading.Thread(target=self._run_vision, daemon=True)
        self._vision_thread.start()
        self.get_logger().info('Vision LCD publisher started (headless)')

    def _on_count(self, config_name: str, total_count: dict):
        """Called when a plant is counted. Publish LCD update."""
        color = LCD_COLORS.get(config_name, DEFAULT_COLOR)
        self._publish_color(color)
        self._publish_text(_format_counts(total_count))
        self.get_logger().info(
            f'Count: {config_name} -> {_format_counts(total_count)}'
        )

    def _publish_text(self, text: str):
        msg = String()
        msg.data = text
        self.text_pub.publish(msg)

    def _publish_color(self, color: tuple):
        msg = ColorRGBA()
        msg.r, msg.g, msg.b, msg.a = color
        self.color_pub.publish(msg)

    def _run_vision(self):
        """Run the vision camera loop in a background thread."""
        try:
            self._vision_app.runApplication()
        except Exception as e:
            self.get_logger().error(f'Vision loop error: {e}')

    def shutdown(self):
        """Stop the vision application."""
        self._vision_app.running = False
        if hasattr(self._vision_app, 'hsvDataBase'):
            try:
                self._vision_app.hsvDataBase.commitAndClose()
            except Exception:
                pass
        for cam in getattr(self._vision_app, 'cameraList', {}).values():
            if hasattr(cam, 'cap') and cam.cap:
                try:
                    cam.cap.release()
                except Exception:
                    pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionLCDPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

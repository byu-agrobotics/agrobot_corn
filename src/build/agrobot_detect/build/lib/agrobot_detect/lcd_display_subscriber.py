#!/usr/bin/env python3
"""
LCD Display Subscriber Node

Subscribes to lcd_display/text and lcd_display/color topics, then displays
the text on the Waveshare 2-inch LCD with the specified background color.
"""

import os
import sys

# Add lcdpractice to path so we can import lib.LCD_2inch
# (Must detect Waveshare lib/LCD_2inch.py — not ROS install/.../lib/python3.x/...)
def _lcdpractice_has_vendor_lib(root):
    return os.path.isfile(os.path.join(root, 'lib', 'LCD_2inch.py'))

_script_dir = os.path.dirname(os.path.abspath(__file__))
_lcdpractice_root = os.environ.get('LCDPRACTICE_ROOT')
if not _lcdpractice_root or not _lcdpractice_has_vendor_lib(_lcdpractice_root):
    _lcdpractice_root = _script_dir
    for _ in range(12):
        if _lcdpractice_has_vendor_lib(_lcdpractice_root):
            break
        _lcdpractice_root = os.path.dirname(_lcdpractice_root)
if _lcdpractice_has_vendor_lib(_lcdpractice_root) and _lcdpractice_root not in sys.path:
    sys.path.insert(0, _lcdpractice_root)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from PIL import Image, ImageDraw, ImageFont


class LCDDisplaySubscriber(Node):
    """Subscribes to display topics and renders to the Waveshare LCD."""

    def __init__(self):
        super().__init__('lcd_display_subscriber')
        self._text = 'Waiting...'
        self._color = (0.1, 0.1, 0.44, 1.0)  # Default navy blue (r, g, b, a)
        self._display = None
        self._font = None
        self._initialized = False

        self.text_sub = self.create_subscription(
            String, 'lcd_display/text', self.text_callback, 10)
        self.color_sub = self.create_subscription(
            ColorRGBA, 'lcd_display/color', self.color_callback, 10)

    def _init_display(self):
        """Lazily initialize the LCD (requires hardware)."""
        if self._initialized:
            return True
        try:
            from lib.LCD_2inch import LCD_2inch
            self._display = LCD_2inch()
            self._display.Init()
            self._display.bl_DutyCycle(100)
            try:
                self._font = ImageFont.truetype(
                    '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 24)
            except OSError:
                self._font = ImageFont.load_default()
            self._initialized = True
            self.get_logger().info('LCD display initialized')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to init LCD: {e}')
            return False

    def text_callback(self, msg):
        """Handle incoming text message."""
        self._text = msg.data
        self.get_logger().info(f'Received text: "{self._text}"')
        self._update_display()

    def color_callback(self, msg):
        """Handle incoming color message."""
        self._color = (msg.r, msg.g, msg.b, msg.a)
        self.get_logger().info(f'Received color: ({msg.r:.2f}, {msg.g:.2f}, {msg.b:.2f})')
        self._update_display()

    def _color_to_rgb255(self):
        """Convert ColorRGBA (0-1) to PIL RGB tuple (0-255)."""
        r, g, b, _ = self._color
        return (int(r * 255), int(g * 255), int(b * 255))

    def _update_display(self):
        """Redraw the LCD with current text and background color."""
        if not self._init_display():
            return

        width, height = 240, 320
        bg_color = self._color_to_rgb255()
        image = Image.new('RGB', (width, height), bg_color)
        draw = ImageDraw.Draw(image)

        # Get text size for centering
        try:
            bbox = draw.textbbox((0, 0), self._text, font=self._font)
            text_width = bbox[2] - bbox[0]
            text_height = bbox[3] - bbox[1]
        except AttributeError:
            text_width, text_height = draw.textsize(self._text, font=self._font)

        x = (width - text_width) // 2
        y = (height - text_height) // 2

        draw.text((x, y), self._text, fill=(255, 255, 255), font=self._font)
        self._display.ShowImage(image)

    def shutdown(self):
        """Clean up display on exit."""
        if self._display is not None:
            try:
                self._display.module_exit()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LCDDisplaySubscriber()
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

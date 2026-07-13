#!/usr/bin/env python3
"""
Object Detect Publisher Node

Runs the plant-detection pipeline from VisionApplication and announces WHAT the
camera is currently seeing on a ROS2 topic, the moment an object is confirmed on
screen (not tied to any counting line). This is the perception -> rest-of-robot
bridge: nav / actuation can subscribe to learn "what am I looking at right now".

One announcement is published per object, the first time its track is stable
(>= MIN_TRACK_AGE frames). The three corn-stalk forms are reported as:

    base (off-center) -> "base:not_centered"
    base (centered)   -> "base:centered"
    single green      -> "green_1_stalk"
    double stalk      -> "double_stalk:yellow_first"  (yellow stalk is first)
                         "double_stalk:green_first"   (green stalk is first)

A double stalk always has a yellow stalk over a green stalk; "which is first" is
derived from the yellow center's horizontal position relative to the green center.
By default the LEFTMOST stalk is called "first" -- flip it with the `leading_side`
parameter ("left" or "right") without touching code.

A base reports whether it is centered under the camera. Unlike the other classes
(one announcement each), a base re-announces every time it crosses into or out of
the centered zone, so nav/actuation get a live "the base is lined up now" signal.
The centered band is set by the `center_tolerance` parameter (fraction of frame
width from center; 0.15 -> middle 30% of the frame).

Topic (default): detected_object   (std_msgs/String)
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from agrobot_detect.vision_module import VisionApplication


class ObjectDetectPublisher(Node):
    """Runs the vision pipeline and announces detected objects as they enter frame."""

    def __init__(self):
        super().__init__('object_detect_publisher')

        # --- Parameters ------------------------------------------------------
        # Which side of the frame counts as "first". Default: leftmost stalk is
        # first. Set to "right" if the robot's travel direction makes the right
        # stalk the leading one.
        self.declare_parameter('leading_side', 'left')
        self.declare_parameter('topic', 'detected_object')
        # A base is "centered" when its center is within this fraction of the frame
        # width from the horizontal center (0.15 -> middle 30% of the frame).
        self.declare_parameter('center_tolerance', 0.15)
        # Serve an MJPEG debug view of the annotated frames at
        # http://<pi-ip>:<stream_port>/stream. Disable when running alongside
        # camera_node (which also binds 8080) or to save CPU.
        self.declare_parameter('streaming_video', True)
        self.declare_parameter('stream_port', 8080)

        self._leading_side = (
            self.get_parameter('leading_side').get_parameter_value().string_value
            or 'left'
        ).lower()
        if self._leading_side not in ('left', 'right'):
            self.get_logger().warn(
                f"leading_side='{self._leading_side}' invalid; falling back to 'left'."
            )
            self._leading_side = 'left'

        topic = (
            self.get_parameter('topic').get_parameter_value().string_value
            or 'detected_object'
        )
        self._center_tolerance = (
            self.get_parameter('center_tolerance').get_parameter_value().double_value
        )
        self._streaming_video = (
            self.get_parameter('streaming_video').get_parameter_value().bool_value
        )
        self._stream_port = (
            self.get_parameter('stream_port').get_parameter_value().integer_value
        )
        self._pub = self.create_publisher(String, topic, 10)

        # --- Vision pipeline -------------------------------------------------
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
            on_detect_callback=self._on_detect,
            center_tolerance=self._center_tolerance,
            streaming_video=self._streaming_video,
            stream_port=self._stream_port,
            use_gui=False,
        )

        self._vision_thread = threading.Thread(target=self._run_vision, daemon=True)
        self._vision_thread.start()
        stream_note = (
            f"stream=http://<pi-ip>:{self._stream_port}/stream"
            if self._streaming_video else "stream=off"
        )
        self.get_logger().info(
            f"Object detect publisher started; topic='{topic}', "
            f"leading_side='{self._leading_side}', {stream_note}"
        )

    def _first_stalk(self, meta: dict) -> str:
        """Map the raw yellow_side geometry to which stalk is 'first'."""
        yellow_side = meta.get('yellow_side')
        if yellow_side not in ('left', 'right'):
            return ''
        # The stalk on the leading side is first.
        return 'yellow' if yellow_side == self._leading_side else 'green'

    def _on_detect(self, config_name: str, meta: dict):
        """Called when an object is confirmed on screen.

        Fires once per object for green_1_stalk / double_stalk. For a base it also
        fires again each time the base crosses into or out of the centered zone.
        """
        payload = config_name
        if config_name == 'double_stalk':
            first = self._first_stalk(meta)
            if first:
                payload = f'double_stalk:{first}_first'
        elif config_name == 'base':
            centered = meta.get('centered')
            if centered is True:
                payload = 'base:centered'
            elif centered is False:
                payload = 'base:not_centered'

        msg = String()
        msg.data = payload
        self._pub.publish(msg)
        self.get_logger().info(f'Detected: {payload}')

    def _run_vision(self):
        """Run the vision camera loop in a background thread."""
        try:
            self._vision_app.runApplication()
        except Exception as e:
            self.get_logger().error(f'Vision loop error: {e}')

    def shutdown(self):
        """Stop the vision application and release resources."""
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
    node = ObjectDetectPublisher()
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

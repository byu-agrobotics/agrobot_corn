#!/usr/bin/env python3
"""
Object Detect Publisher Node

Runs the plant-detection pipeline from VisionApplication and announces WHAT the
camera is currently seeing on a ROS2 topic, the moment an object is confirmed on
screen (not tied to any counting line). This is the perception -> rest-of-robot
bridge: nav / actuation can subscribe to learn "what am I looking at right now".

Two things are published, each with a centered / not_centered state:

    base         -> "base:centered" / "base:not_centered"          (stop to seed)
    yellow stalk -> "yellow_stalk:centered" / "yellow_stalk:not_centered"
                                                        (stop to remove the stalk)

A lone green stalk (green_1_stalk) is NOT published. "yellow_stalk" fires for any
yellow stalk the robot lines up on to remove, whether or not a green stalk sits
under it: a double stalk (yellow over green) is centered on the yellow stalk's own
position carried in the track meta, and a standalone yellow stalk (yellow_1_stalk,
no green under it) is centered on its own centroid -- both are the yellow stalk's
center.

Both signals re-announce every time the object crosses into or out of the centered
zone, so nav/actuation get a live "it is lined up now" edge. The centered band is
set by the `center_tolerance` parameter (fraction of frame width from center;
0.15 -> middle 30% of the frame).

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
        self.declare_parameter('topic', 'detected_object')
        # A base is "centered" when its center is within this fraction of the frame
        # width from the horizontal center (0.15 -> middle 30% of the frame).
        self.declare_parameter('center_tolerance', 0.20)
        # Reject a 'base' detected in the upper part of the frame (false positive:
        # bases sit low in view). 0.5 -> ignore any base in the top half.
        self.declare_parameter('base_ignore_above', 0.5)
        # Serve an MJPEG debug view of the annotated frames at
        # http://<pi-ip>:<stream_port>/stream. Disable when running alongside
        # camera_node (which also binds 8080) or to save CPU.
        self.declare_parameter('streaming_video', True)
        self.declare_parameter('stream_port', 8080)

        topic = (
            self.get_parameter('topic').get_parameter_value().string_value
            or 'detected_object'
        )
        self._center_tolerance = (
            self.get_parameter('center_tolerance').get_parameter_value().double_value
        )
        self._base_ignore_above = (
            self.get_parameter('base_ignore_above').get_parameter_value().double_value
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
            base_ignore_above=self._base_ignore_above,
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
            f"Object detect publisher started; topic='{topic}', {stream_note}"
        )

    # Which detected class maps to which published label. Anything not listed
    # here (e.g. green_1_stalk) is intentionally not published.
    _LABELS = {
        'base': 'base',
        'double_stalk': 'yellow_stalk',
        # A yellow stalk with no green stalk under it (not a double) still aligns
        # on the yellow stalk, so publish it as yellow_stalk too.
        'yellow_1_stalk': 'yellow_stalk',
    }

    def _on_detect(self, config_name: str, meta: dict):
        """Called when a base or a double stalk (yellow stalk) is on screen.

        Publishes "<label>:centered" / "<label>:not_centered" and re-fires each
        time the object crosses into or out of the centered zone. green_1_stalk is
        dropped.
        """
        label = self._LABELS.get(config_name)
        if label is None:
            return  # green_1_stalk (and anything else) is not published

        centered = meta.get('centered')
        if centered is True:
            payload = f'{label}:centered'
        elif centered is False:
            payload = f'{label}:not_centered'
        else:
            payload = label  # defensive: centering state unknown

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

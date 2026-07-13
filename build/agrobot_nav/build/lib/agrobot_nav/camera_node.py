"""
ROS2 Camera Node for Arducam UC-684

Captures frames from the USB camera using OpenCV, publishes them as
sensor_msgs/Image on /camera/image_raw, and serves a live MJPEG stream
over HTTP so the feed can be viewed in a browser when SSH'd in.

Usage:
    ros2 run agrobot_nav camera
    Then open http://<pi-ip>:8080 in your browser.
"""

import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP request handler that streams MJPEG from the latest frame."""

    # Class-level reference set by CameraNode before the server starts
    latest_frame: Optional[np.ndarray] = None
    frame_lock = threading.Lock()
    # Seconds to sleep per stream iteration; set from the fps param by CameraNode.
    # Without this the /stream loop re-encodes the same frame as fast as the CPU
    # allows, pegging a core and starving sshd whenever the feed is being viewed.
    frame_interval = 1.0 / 30.0

    def do_GET(self):
        if self.path == '/':
            # Serve a minimal HTML page that shows the stream
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            html = b"""\
<!DOCTYPE html>
<html>
<head>
    <title>Agrobot Camera</title>
    <style>
        body {
            margin: 0;
            background: #111;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
            font-family: sans-serif;
            color: #eee;
        }
        h1 { margin-bottom: 12px; font-weight: 300; letter-spacing: 2px; }
        img {
            max-width: 95vw;
            max-height: 85vh;
            border: 2px solid #333;
            border-radius: 8px;
        }
    </style>
</head>
<body>
    <h1>Agrobot Camera Feed</h1>
    <img src="/stream" alt="Camera Feed" />
</body>
</html>
"""
            self.wfile.write(html)

        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            try:
                import time
                while True:
                    with MJPEGHandler.frame_lock:
                        frame = MJPEGHandler.latest_frame
                    if frame is None:
                        time.sleep(0.01)
                        continue

                    ret, jpeg = cv2.imencode(
                        '.jpg', frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ret:
                        continue

                    data = jpeg.tobytes()
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(
                        b'Content-Type: image/jpeg\r\n'
                        b'Content-Length: ' + str(len(data)).encode() + b'\r\n'
                        b'\r\n')
                    self.wfile.write(data)
                    self.wfile.write(b'\r\n')

                    # Pace to the camera fps instead of spinning flat-out.
                    time.sleep(MJPEGHandler.frame_interval)
            except (BrokenPipeError, ConnectionResetError):
                pass  # Client disconnected

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        """Suppress default HTTP log noise — the ROS logger handles it."""
        pass


class CameraNode(Node):
    """Captures frames from a USB camera, publishes to ROS and streams MJPEG."""

    def __init__(self):
        super().__init__('camera_node')

        # ---- Declare parameters ------------------------------------------------
        self.declare_parameter('device_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('web_port', 8080)

        device_index = self.get_parameter('device_index').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        web_port = self.get_parameter('web_port').value

        # ---- Open camera -------------------------------------------------------
        self.cap = cv2.VideoCapture(device_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open camera at /dev/video{device_index}. '
                'Try a different device_index parameter.')
            raise RuntimeError('Cannot open camera')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'Camera opened: /dev/video{device_index} @ {actual_w}x{actual_h}')

        # ---- ROS publisher ------------------------------------------------------
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # ---- Web server (runs in a background daemon thread) --------------------
        # Pace the /stream loop to the camera fps so it can't peg a CPU core.
        MJPEGHandler.frame_interval = 1.0 / fps if fps > 0 else 1.0 / 30.0
        self.http_server = HTTPServer(('0.0.0.0', web_port), MJPEGHandler)
        self.server_thread = threading.Thread(
            target=self.http_server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(
            f'MJPEG web stream available at http://0.0.0.0:{web_port}')

        # ---- Timer drives the capture loop --------------------------------------
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self._capture_callback)

    # ---------------------------------------------------------------------- #
    def _capture_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        # Update the frame shared with the web server
        with MJPEGHandler.frame_lock:
            MJPEGHandler.latest_frame = frame

        # Publish as sensor_msgs/Image (raw BGR8, no cv_bridge needed)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = False
        msg.step = frame.shape[1] * 3  # 3 bytes per pixel (BGR)
        msg.data = frame.tobytes()

        self.image_pub.publish(msg)

    # ---------------------------------------------------------------------- #
    def destroy_node(self):
        self.get_logger().info('Shutting down camera node …')
        self.http_server.shutdown()
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

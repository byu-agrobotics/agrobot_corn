#!/usr/bin/env python3
"""
Simple camera stream server for Raspberry Pi.
Serves an MJPEG stream over HTTP so a laptop can connect and use it
(e.g. with OpenCV: cv2.VideoCapture('http://<PI_IP>:8080/stream')).

Run on the Pi:
    python stream_pi_camera.py
    python stream_pi_camera.py --port 8080
    python stream_pi_camera.py --port 8080 --width 640 --height 480

Then on your laptop, open in Vision Calibration or OpenCV:
    http://<PI_IP>:8080/stream
"""

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Prefer Pi Camera (CSI) on Raspberry Pi; fall back to USB via OpenCV
try:
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False

import cv2


class CameraSource:
    """Shared camera source used by all HTTP clients."""

    def __init__(self, width=640, height=480, use_pi_camera=True, camera_index=0):
        self.width = width
        self.height = height
        self.use_pi_camera = use_pi_camera
        self.camera_index = camera_index
        self._lock = threading.Lock()
        self._picam2 = None
        self._cap = None

    def start(self):
        if self.use_pi_camera:
            self._picam2 = Picamera2(camera_num=self.camera_index)
            config = self._picam2.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            self._picam2.configure(config)
            self._picam2.start()
            return

        # USB or default: OpenCV
        self._cap = cv2.VideoCapture(self.camera_index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self._cap.isOpened():
            raise RuntimeError(f"Failed to open USB camera index {self.camera_index}")

    def read_frame_bgr(self):
        """Return latest frame in BGR format, or None on failure."""
        with self._lock:
            if self._picam2 is not None:
                frame = self._picam2.capture_array()
                if frame is None:
                    return None
                # RGB from picamera2 -> BGR for OpenCV JPEG encoding
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            if self._cap is not None:
                ok, frame = self._cap.read()
                if not ok:
                    return None
                return frame
        return None

    def stop(self):
        with self._lock:
            if self._picam2 is not None:
                self._picam2.stop()
                self._picam2 = None
            if self._cap is not None:
                self._cap.release()
                self._cap = None


class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"ok\n")
            return

        if self.path == "/" or self.path == "/stream":
            try:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
            except (BrokenPipeError, ConnectionResetError):
                return

            try:
                while True:
                    frame = self.server.camera.read_frame_bgr()
                    if frame is None:
                        time.sleep(0.03)
                        continue
                    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(buf)}\r\n\r\n".encode("ascii"))
                    self.wfile.write(buf.tobytes())
                    self.wfile.write(b"\r\n")
                    time.sleep(0.033)  # ~30 fps
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Reduce log noise
        pass


def main():
    parser = argparse.ArgumentParser(description="Stream Pi camera over HTTP (MJPEG)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Port (default: 8080)")
    parser.add_argument("--width", type=int, default=640, help="Frame width (default: 640)")
    parser.add_argument("--height", type=int, default=480, help="Frame height (default: 480)")
    parser.add_argument(
        "--camera",
        type=int,
        default=0,
        help="Camera index (default: 0). For Pi dual cameras use 0 or 1.",
    )
    parser.add_argument(
        "--usb",
        action="store_true",
        help="Use USB camera (OpenCV) instead of Pi Camera",
    )
    args = parser.parse_args()

    use_pi_camera = not args.usb and HAS_PICAMERA2
    if args.usb:
        print("Using USB camera (OpenCV)")
    elif HAS_PICAMERA2:
        print("Using Pi Camera (picamera2)")
    else:
        print("picamera2 not found; using USB camera (OpenCV)")

    camera = CameraSource(
        width=args.width,
        height=args.height,
        use_pi_camera=use_pi_camera,
        camera_index=args.camera,
    )
    camera.start()

    server = ThreadingHTTPServer((args.host, args.port), StreamingHandler)
    server.camera = camera

    print(f"Streaming at http://<PI_IP>:{args.port}/stream")
    print(f"Health check: http://<PI_IP>:{args.port}/health")
    print("On your laptop, use: cv2.VideoCapture('http://<PI_IP>:8080/stream')")
    print("Press Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        camera.stop()


if __name__ == "__main__":
    main()

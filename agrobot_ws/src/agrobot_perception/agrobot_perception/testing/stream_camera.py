#!/usr/bin/env python3
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer
import time

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            cap = cv2.VideoCapture(0)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            try:
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        break
                    
                    frame = cv2.resize(frame, (640, 480))
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    frame_bytes = buffer.tobytes()
                    
                    self.wfile.write(b'--frame\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame_bytes))
                    self.end_headers()
                    self.wfile.write(frame_bytes)
                    self.wfile.write(b'\r\n')
                    
                    time.sleep(0.033)
            except:
                pass
            finally:
                cap.release()
        else:
            self.send_response(404)
            self.end_headers()

if __name__ == '__main__':
    server = HTTPServer(('0.0.0.0', 8080), StreamingHandler)
    print('Streaming at http://0.0.0.0:8080/stream')
    server.serve_forever()

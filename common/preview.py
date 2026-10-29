"""Tiny MJPEG preview server: http://<jetson>:8080/ shows the latest annotated frame."""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2


class Preview:
    def __init__(self, port=8080, scale=0.5):
        self.scale, self._jpeg, self._lock = scale, None, threading.Lock()
        preview = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=f")
                self.end_headers()
                try:
                    while True:
                        with preview._lock:
                            b = preview._jpeg
                        if b:
                            self.wfile.write(b"--f\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(b)
                                             + b + b"\r\n")
                        time.sleep(1 / 10)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def show(self, frame):
        small = cv2.resize(frame, None, fx=self.scale, fy=self.scale)
        ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with self._lock:
                self._jpeg = buf.tobytes()

    def close(self):
        self.server.shutdown()

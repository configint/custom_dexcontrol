#!/usr/bin/env python3
"""
USB camera MJPEG stream server.
View in your browser at http://<robot-ip>:8080
"""

import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2

PORT = 8080
FPS = 15

# Camera devices that are working
CAMERAS = {
    "video0": "/dev/video0",
    "video2": "/dev/video2",
    "video4": "/dev/video4",
    "video6": "/dev/video6",
}

latest_frames = {}
frame_lock = threading.Lock()
running = True


def capture_loop(name, device):
    """Continuously capture frames from a USB camera."""
    global running
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open {name} ({device})")
        return
    print(f"[OK] {name} opened ({device})")

    while running:
        ret, frame = cap.read()
        if ret:
            with frame_lock:
                latest_frames[name] = frame
        time.sleep(1.0 / FPS)

    cap.release()


def encode_frame(frame):
    """Encode a BGR frame to JPEG bytes."""
    if frame is None:
        return None
    _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    return buf.tobytes()


INDEX_HTML = """\
<!DOCTYPE html>
<html>
<head>
  <title>Robot Camera Stream</title>
  <style>
    body {{ background: #1a1a1a; color: #eee; font-family: sans-serif;
           margin: 0; padding: 20px; text-align: center; }}
    h1 {{ margin-bottom: 20px; }}
    .streams {{ display: flex; flex-wrap: wrap; justify-content: center; gap: 16px; }}
    .stream {{ background: #222; border-radius: 8px; padding: 10px; }}
    .stream h3 {{ margin: 0 0 8px 0; }}
    img {{ max-width: 640px; width: 100%; border-radius: 4px; }}
  </style>
</head>
<body>
  <h1>Robot Camera Stream</h1>
  <div class="streams">
    {streams}
  </div>
</body>
</html>
"""


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            streams_html = ""
            for name in CAMERAS:
                streams_html += (
                    f'<div class="stream"><h3>{name}</h3>'
                    f'<img src="/stream/{name}"></div>\n'
                )
            html = INDEX_HTML.format(streams=streams_html)
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(html.encode())

        elif self.path.startswith("/stream/"):
            key = self.path.split("/stream/")[-1]
            if key not in CAMERAS:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header(
                "Content-Type", "multipart/x-mixed-replace; boundary=frame"
            )
            self.end_headers()
            while running:
                with frame_lock:
                    raw = latest_frames.get(key)
                if raw is None:
                    time.sleep(0.05)
                    continue
                jpg = encode_frame(raw)
                if jpg is None:
                    time.sleep(0.05)
                    continue
                try:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                except BrokenPipeError:
                    break
                time.sleep(1.0 / FPS)
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def main():
    global running

    # Start capture threads
    threads = []
    for name, device in CAMERAS.items():
        t = threading.Thread(target=capture_loop, args=(name, device), daemon=True)
        t.start()
        threads.append(t)

    server = ThreadedHTTPServer(("0.0.0.0", PORT), StreamHandler)

    def shutdown(sig, frame):
        global running
        print("\nShutting down...")
        running = False
        import os

        os._exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"\n  Open in your browser: http://<robot-ip>:{PORT}")
    print("  Robot IPs: 192.168.50.20 / 192.168.68.60")
    print("  Press Ctrl+C to stop.\n")
    server.serve_forever()


if __name__ == "__main__":
    main()

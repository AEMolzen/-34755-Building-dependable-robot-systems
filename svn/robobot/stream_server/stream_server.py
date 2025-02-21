#!/usr/bin/env python3
# Rui Santos & Sara Santos - Random Nerd Tutorials
# Complete project details at https://RandomNerdTutorials.com/raspberry-pi-mjpeg-streaming-web-server-picamera2/

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:7123
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import io
import logging
import socketserver
import socket
from http import server
from threading import Condition
from setproctitle import setproctitle
#
from picamera2 import Picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

# set title of process, so that it is not just called Python
setproctitle("stream_server")

hostname = socket.gethostname()
PAGE = """\
<html>
<head>
<title>picamera2 MJPEG</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming from {}</h1>
<img src="stream.mjpg" width="820" height="616" />
<!--img src="stream.mjpg" width="1296" height="972" -->
<!--img src="stream.mjpg" width="320" height="240" -->
<!--img src="stream.mjpg" width="640" height="480" -->
</body>
</html>
""".format(hostname)

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True

# # 1296x972
picam2 = Picamera2()
#picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))
# higher resolution and lower framerate (5 FPS (200000 microseconds between frames))
picam2.configure(picam2.create_video_configuration(main={"size": (820, 616)},controls={'FrameDurationLimits': (200000, 500000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (1296, 972)},controls={'FrameDurationLimits': (200000, 500000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (1296, 972)},controls={'FrameDurationLimits': (200000, 200000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (1296, 972)},controls={'FrameDurationLimits': (50000, 200000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)},controls={'FrameDurationLimits': (200000, 500000)}))
#picam2.configure(picam2.create_video_configuration(main={"size": (320, 240)},controls={'FrameDurationLimits': (200000, 500000)}))
output = StreamingOutput()
picam2.start_recording(JpegEncoder(), FileOutput(output))

try:
    address = ('', 7123)
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()
finally:
    picam2.stop_recording()

from lib.Vision import Vision
from lib.interface import WSServer
import cv2

vision = Vision()
vision.start()

server = WSServer()
server.run()

try:
    while True:
        vision.wait_next_frame()
        frame = vision.camera.latest_frame
        x, y, r = vision.ball_info_v[2:]
        stream_frame = cv2.cvtColor(cv2.circle(frame, (x, y), r, (150, 50, 0), 10, cv2.LINE_AA), cv2.COLOR_RGB2BGR)
        server.send_frame(stream_frame)
        print(f"Streamed at {x, y, r}")

except KeyboardInterrupt:
    vision.deinit()
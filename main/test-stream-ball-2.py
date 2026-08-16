SEND_FRAME = True

from lib.config import Config
from lib.imu import IMU
from lib.Vision import Vision
from lib.drive import Drive
from lib.interface import WSServer

import time
import cv2
import numpy as np

server = WSServer()
server.run()

vision = Vision()
vision.start()
R = min(vision.camera.size) / 2

imu = IMU()
config = Config()

drive = Drive.from_config(config)

# Font settings
default_font = (cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)
padx = 15
pady = 25
line_spacing = 30

while True:
    try:
        bangle, bdist, bx, by, br = vision.ball_info
        # gbang1, gbang2, gbzero, gbx, gby = vision.bgoal_info_v
        # gyang1, gyang2, gyzero, gyx, gyy = vision.ygoal_info_v

        if SEND_FRAME:
            vision.wait_next_frame()
            frame = vision.camera.latest_frame
            center = (frame.shape[1] // 2 - 20, frame.shape[0] // 2 + 55)
    
            # Show ball
            frame = cv2.line(frame, (center[0], center[1] - 5), (center[0], center[1] + 5), (255, 0, 0), 1, cv2.LINE_AA)
            frame = cv2.line(frame, (center[0] - 5, center[1]), (center[0] + 5, center[1]), (255, 0, 0), 1, cv2.LINE_AA)
            frame = cv2.circle(frame, (int(bx), int(by)), br, (0, 50, 150), 8, cv2.LINE_AA)
            frame = cv2.putText(frame, f"Ball angle: {bangle}", (padx, pady), *default_font)
            frame = cv2.putText(frame, f"Ball distance: {bdist}", (padx, pady+line_spacing), *default_font)
    
            # Send frame
            server.send_frame(frame)
        # Move
        # drive.move(bangle, 0.1)
        print(bangle)
    except KeyboardInterrupt:
        break

drive.stop()
vision.deinit()

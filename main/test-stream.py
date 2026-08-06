# TODO:
# Add black box behind text
# Double check possible coord flipping and atan2

from lib.Vision import Vision
from lib.interface import WSServer
import cv2
import numpy as np
import time

vision = Vision()
vision.start()
R = min(vision.camera.size) / 2

server = WSServer()
server.run()

last_time = time.time()

# Font settings
default_font = (cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)
padx = 15
pady = 25
line_spacing = 30

# Everything handler (lol idk how to actl implement)
# all config values from 0 to 255
# all bounds are 3 values long, goal_ke is a single value
def img_cfg_change(cfg: dict):
    ...

try:
    while True:
        vision.wait_next_frame()
        frame = vision.camera.latest_frame
        bangle, bdist, bx, by, br = vision.ball_info_v
        gbang1, gbang2, gbzero, gbx, gby = vision.bgoal_info_v
        gyang1, gyang2, gyzero, gyx, gyy = vision.ygoal_info_v

        # Show ball
        frame = cv2.circle(frame, (bx, by), br, (0, 50, 150), 8, cv2.LINE_AA)
        frame = cv2.putText(frame, f"Ball angle: {bangle}", (padx, pady), *default_font)
        frame = cv2.putText(frame, f"Ball distance: {bdist}", (padx, pady+line_spacing), *default_font)

        # Show bgoal
        frame = cv2.circle(frame, (gbx, gby), 10, (200, 50, 50), 10, cv2.LINE_AA)
        frame = cv2.line(frame, vision.camera.center, (int(R*np.cos(gbang1 / 32767 * np.pi)), int(R*np.sin(gbang1 / 32767 * np.pi))), (200, 50, 50), 8, cv2.LINE_AA)
        frame = cv2.line(frame, vision.camera.center, (int(R*np.cos(gbang2 / 32767 * np.pi)), int(R*np.sin(gbang2 / 32767 * np.pi))), (200, 50, 50), 8, cv2.LINE_AA)
        frame = cv2.putText(frame, f"Blue goal aim: {["OK", "Turn left", "Turn right"][gbzero]}", (padx, pady+2*line_spacing), *default_font)

        # Show ygoal
        frame = cv2.circle(frame, (gyx, gyy), 10, (0, 180, 250), 10, cv2.LINE_AA)
        frame = cv2.line(frame, vision.camera.center, (int(R*np.cos(gyang1 / 32767 * np.pi)), int(R*np.sin(gyang1 / 32767 * np.pi))), (0, 180, 250), 8, cv2.LINE_AA)
        frame = cv2.line(frame, vision.camera.center, (int(R*np.cos(gyang2 / 32767 * np.pi)), int(R*np.sin(gyang2 / 32767 * np.pi))), (0, 180, 250), 8, cv2.LINE_AA)
        frame = cv2.putText(frame, f"Yellow goal aim: {["OK", "Turn left", "Turn right"][gyzero]}", (padx, pady+3*line_spacing), *default_font)
        
        server.send_frame(frame)
        # exp = float(input("Hi: "))
        # vision.camera.camera.set_controls({"LensPosition": exp})
        now_time = time.time()
        # cv2.imwrite("/var/www/html/frame.jpg", frame)
        print(f"Streamed {round(now_time - last_time, 2)} s later")
        last_time = now_time
except KeyboardInterrupt:
    vision.deinit()

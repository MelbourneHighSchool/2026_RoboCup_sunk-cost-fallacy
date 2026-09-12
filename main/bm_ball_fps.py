from lib.Vision import Vision
import numpy as np
import time

vision = Vision()
vision.start()
prev_frame = None
time_last = time.time()

while True:
    ball_diff = (vision.ball_time_v[-1] - vision.ball_time_v[-2])
    if ball_diff == 0:
        continue
    goal_diff = (vision.goal_time_v[-1] - vision.goal_time_v[-2])
    if goal_diff == 0:
        continue
    print(f"Ball FPS: {1/ball_diff:.2f}, Goal FPS: {1/goal_diff:.2f}")
    time.sleep(0.033)
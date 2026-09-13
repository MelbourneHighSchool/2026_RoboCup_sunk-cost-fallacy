from lib.Vision import Vision
import numpy as np
import time

vision = Vision()
vision.start()
prev_frame = None
time_last = time.time()

while True:
    ball_diffs = [vision.ball_time_v[i] - vision.ball_time_v[i-1] for i in range(1, len(vision.ball_time_v))]
    goal_diffs = [vision.goal_time_v[i] - vision.goal_time_v[i-1] for i in range(1, len(vision.goal_time_v))]

    if sum(ball_diffs) == 0 or sum(goal_diffs) == 0:
        continue

    print(f"Ball FPS: {1/(sum(ball_diffs)/len(ball_diffs)):.2f}, Goal FPS: {1/(sum(goal_diffs)/len(goal_diffs)):.2f}")
    time.sleep(0.033)
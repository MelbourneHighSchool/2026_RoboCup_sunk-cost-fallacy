"""Test the camera and stream its output to the interface"""
from lib.Vision import Vision
from lib.interface import WSServer

import cv2
import numpy as np
import time
import random

R = 1000
def main(config):
    vision = Vision()
    vision.center_v[0] += -10
    vision.center_v[1] += 26
    vision.start()
    vision.load_config(config)

    server = WSServer()
    server.run()

    last_center_time = time.time()

    vision.wait_next_frame()
    while True:
        try:
            if time.time() >= last_center_time + 10:
                vision.center_v[0] = random.randint(60, 580)
                vision.center_v[1] = random.randint(60, 580)
                last_center_time += 10
            frame = vision.camera.latest_frame

            center = vision.center_v[:]

            # Draw ball
            b_angle, b_dist, b_x, b_y, b_r = vision.ball_info

            frame = cv2.line(frame, (center[0], center[1]), (b_x, b_y), (50, 100, 255), 2, cv2.LINE_AA)
            frame = cv2.circle(frame, (b_x, b_y), b_r, (50, 100, 255), 2, cv2.LINE_AA)

            # Draw blue goal
            bg_raw_angle, bg_raw_ang_width, bg_dist, bg_x, bg_y, bg_w, bg_h, bg_rot = vision.bgoal_info_v
            bg_angle = -(bg_raw_angle * 180 / 32767 + 90) % 360
            bg_ang_width = bg_raw_ang_width * 180 / 32767

            if bg_dist:
                # frame = cv2.line(frame, (center[0], center[1]), (bg_x, bg_y), (255, 100, 50), 2, cv2.LINE_AA)
                bg_left_angle = np.radians(bg_angle + bg_ang_width)
                bg_right_angle = np.radians(bg_angle - bg_ang_width)
                frame = cv2.line(frame, (center[0], center[1]), (int(center[0]-R*np.sin(bg_left_angle)), int(center[1]-R*np.cos(bg_left_angle))), (255, 100, 50), 1, cv2.LINE_AA)
                frame = cv2.line(frame, (center[0], center[1]), (int(center[0]-R*np.sin(bg_right_angle)), int(center[1]-R*np.cos(bg_right_angle))), (255, 100, 50), 1, cv2.LINE_AA)
                bg_points = cv2.boxPoints(cv2.RotatedRect((float(bg_x), float(bg_y)), (float(bg_w), float(bg_h)), float(bg_rot))).astype(np.int32)
                frame = cv2.drawContours(frame, [bg_points], 0, (255, 100, 50), 2, cv2.LINE_AA)

            # Draw yellow goal
            yg_raw_angle, yg_raw_ang_width, yg_dist, yg_x, yg_y, yg_w, yg_h, yg_rot = vision.ygoal_info_v
            yg_angle = -(yg_raw_angle * 180 / 32767 + 90) % 360
            yg_ang_width = yg_raw_ang_width * 180 / 32767

            if yg_dist:
                # frame = cv2.line(frame, (center[0], center[1]), (yg_x, yg_y), (50, 255, 255), 2, cv2.LINE_AA)
                yg_left_angle = np.radians(yg_angle + yg_ang_width)
                yg_right_angle = np.radians(yg_angle - yg_ang_width)
                frame = cv2.line(frame, (center[0], center[1]), (int(center[0]-R*np.sin(yg_left_angle)), int(center[1]-R*np.cos(yg_left_angle))), (50, 255, 255), 1, cv2.LINE_AA)
                frame = cv2.line(frame, (center[0], center[1]), (int(center[0]-R*np.sin(yg_right_angle)), int(center[1]-R*np.cos(yg_right_angle))), (50, 255, 255), 1, cv2.LINE_AA)
                yg_points = cv2.boxPoints(cv2.RotatedRect((float(yg_x), float(yg_y)), (float(yg_w), float(yg_h)), float(yg_rot))).astype(np.int32)
                frame = cv2.drawContours(frame, [yg_points], 0, (50, 255, 255), 2, cv2.LINE_AA)

            # Draw center
            frame = cv2.line(frame, (center[0] - 10, center[1]), (center[0] + 10, center[1]), (200, 200, 0), 2, cv2.LINE_AA)
            frame = cv2.line(frame, (center[0], center[1] - 10), (center[0], center[1] + 10), (200, 200, 0), 2, cv2.LINE_AA)

            # Puts text
            frame = cv2.putText(frame, f"Ball angle: {b_angle}", (15, 25), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"Ball dist: {b_dist}", (15, 55), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"BGoal angle: {bg_angle}", (15, 105), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"BGoal dist: {bg_dist}", (15, 135), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"YGoal angle: {yg_angle}", (15, 185), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"YGoal dist: {yg_dist}", (15, 215), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)))

            server.send_frame(frame)
        except KeyboardInterrupt:
            break

    # Cleanup
    vision.deinit()

    return True
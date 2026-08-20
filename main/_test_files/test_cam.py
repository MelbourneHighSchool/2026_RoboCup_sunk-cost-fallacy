"""Test the camera and stream its output to the interface"""
from lib.Vision import Vision
from lib.interface import WSServer

import cv2
import numpy as np

def main(config):
    vision = Vision()
    vision.start()
    vision.load_config(config)

    server = WSServer()
    server.run()

    vision.wait_next_frame()

    while True:
        try:
            frame = vision.camera.latest_frame

            center = (frame.shape[1] // 2 - 20, frame.shape[0] // 2 + 55)

            # Draw ball
            b_angle, b_dist, b_x, b_y, b_r = vision.ball_info

            frame = cv2.line(frame, (center[0], center[1]), (b_x, b_y), (50, 100, 255), 2, cv2.LINE_AA)
            frame = cv2.circle(frame, (b_x, b_y), b_r, (50, 100, 255), 2, cv2.LINE_AA)

            # Draw blue goal
            bg_angle, bg_x, bg_y, bg_w, bg_h, bg_rot = vision.bgoal_info

            frame = cv2.line(frame, (center[0], center[1]), (bg_x, bg_y), (255, 100, 50), 2, cv2.LINE_AA)
            bg_points = cv2.boxPoints(cv2.RotatedRect((float(bg_x), float(bg_y)), (float(bg_w), float(bg_h)), float(bg_rot)))
            frame = cv2.drawContours(frame, [bg_points], 0, (255, 100, 50), 2, cv2.LINE_AA)

            # Draw yellow goal
            yg_angle, yg_x, yg_y, yg_w, yg_h, yg_rot = vision.ygoal_info

            frame = cv2.line(frame, (center[0], center[1]), (yg_x, yg_y), (50, 255, 255), 2, cv2.LINE_AA)
            yg_points = cv2.boxPoints(cv2.RotatedRect((float(yg_x), float(yg_y)), (float(yg_w), float(yg_h)), float(yg_rot)))
            frame = cv2.drawContours(frame, [yg_points], 0, (50, 255, 255), 2, cv2.LINE_AA)

            frame = cv2.putText(frame, f"Ball angle: {b_angle}", (15, 25), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"Ball dist: {b_dist}", (15, 55), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"BGoal angle: {bg_angle}", (15, 85), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))
            frame = cv2.putText(frame, f"YGoal angle: {yg_angle}", (15, 115), *((cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)))

            server.send_frame(frame)
        except KeyboardInterrupt:
            break

    # Cleanup
    vision.deinit()

    return True
"""
TO DO
- Create shared variables to store the config and stuff
- Create the ball process
- Make enemy/teammate/wall/goal shit + process
- Add a way to calibrate mirror centre...

- Possibly add locks for shared variables (but not sure if necessary)
"""

from multiprocessing import Value, Array
from lib.BaseVision import Camera, Broadcaster, BaseProcArgs
from ctypes import c_bool, c_uint8, c_uint16, c_int16, c_float
from typing import Any
import cv2
import numpy as np
import struct
import time

OFFSET_X = -20
OFFSET_Y = 55

def enclosing_interval(angles):
    a = np.sort(angles)
    n = len(a)
    
    # gaps between consecutive sorted angles, plus the wrap-around gap
    gaps = np.diff(a)
    wrap_gap = (a[0] + 2*np.pi) - a[-1]
    all_gaps = np.append(gaps, wrap_gap)
    
    # the largest gap is the "empty" arc - cut there
    max_gap_idx = np.argmax(all_gaps)
    
    if max_gap_idx == n - 1:
        # largest gap is the wrap-around gap itself, so interval doesn't wrap
        start, end = a[0], a[-1]
    else:
        # interval wraps around; start right after the gap, end right before it
        start = a[max_gap_idx + 1]
        end = a[max_gap_idx]
    
    return start, end

class Vision:
    def __init__(self):
        self.camera = Camera()
        self.broadcaster = Broadcaster(self.camera.create_broadcaster_args())

        self.ball_proc_setup()
        self.goal_proc_setup()

    @property
    def ball_info(self):
        angle, distance, x, y, r = self.ball_info_v[:]
        angle = -(angle * 180 / 32767 + 90) % 360
        return (angle, distance, x, y, r)

    @property
    def ygoal_info(self):
        ang, distance, x, y, w, h, rang  = self.ygoal_info_v[:]
        ang = -(ang * 180 / 32767 + 90) % 360
        return (ang, distance, w, h, rang)

    @property
    def bgoal_info(self):
        ang, distance, x, y, w, h, rang  = self.bgoal_info_v[:]
        ang = -(ang * 180 / 32767 + 90) % 360
        return (ang, distance, w, h, rang)

    def load_config(self, config):
        hsv = config.get_value("hsv")
        if hsv is None:
            return False

        self.ball_bounds_v[:] = hsv["ball"]["low"] + hsv["ball"]["high"]
        self.goal_bounds_v[:] = hsv["bgoal"]["low"] + hsv["bgoal"]["high"] + hsv["ygoal"]["low"] + hsv["ygoal"]["high"]
    
    def start(self):
        self.camera.start()
        self.broadcaster.start()

    def deinit(self):
        self.camera.stop()
        self.broadcaster.deinit()

    def wait_next_frame(self, timeout=3):
        "Note: May sometimes trigger without waiting for the next frame"
        with self.camera.c_new_frame:
            self.camera.c_new_frame.wait(timeout=timeout)
    
    # Ball proc
    def ball_proc_setup(self):
        self.ball_bounds_v = Array(c_uint8, (0, 120, 160, 30, 255, 255))  # (lboundH, S, V, uboundH, S, V) - change defaults later!
        self.ball_info_v = Array(c_int16, (0, 0, 0, 0, 0))  # angle, dist, x, y, r

        self.broadcaster.register_proc(
            "Ball",
            self.ball_proc_init,
            self.ball_proc_loop,
            None,
            (
                self.ball_bounds_v,
                self.ball_info_v,
                self.camera.frame_shape
            )
        )
    
    @staticmethod
    def ball_proc_init(ball_bounds_v, ball_info_v, frame_shape):
        mask_frame = np.zeros(shape=frame_shape[:2], dtype=np.uint8)  # 2D array since only 1 channel
        return (ball_bounds_v, ball_info_v, mask_frame)

    @staticmethod
    def ball_proc_loop(base_args: BaseProcArgs, keep_args: dict[str, Any]):
        enabled = base_args.enabled
        if not enabled:
            return
        
        frame_size, frame_shape, latest_idx, latest_timestamp, frame = base_args[:5]
        ball_bounds_v, ball_info_v, mask_frame = keep_args

        # Find ball
        pixel_pos = None
        bounds = np.array(ball_bounds_v, dtype=np.uint8)
        lbound = bounds[:3]
        ubound = bounds[3:]

        # rgb_frame = np.copy(frame)  # DEBUG
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, frame)
        cv2.inRange(frame, lbound, ubound, mask_frame)
        # cv2.imwrite("/var/www/html/frame.jpg", np.hstack((cv2.cvtColor(mask_frame, cv2.COLOR_GRAY2BGR), rgb_frame, frame)))  # DEBUG
        # time.sleep(0.1)  # DEBUG
        ballContours = cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if ballContours:
            bestContour = max(ballContours, key=cv2.contourArea)
            (x, y), r = cv2.minEnclosingCircle(bestContour)
            x, y, r = int(x), int(y), int(r)
            pixel_pos = (x, y)
        
        if not pixel_pos:
            ball_info_v[:] = (0, 0, 0, 0, 0)
            return

        # HI BAO WHY ARE Y AND X SWAPPED TS MAKES 0 SENSE 😭😭😭
        
        center = np.array((frame_shape[1] // 2 + OFFSET_X, frame_shape[0] // 2 + OFFSET_Y), dtype=np.int16)
        img_x, img_y = pixel_pos
        translated_pixel_pos = pixel_pos - center
        distance = min(32767, int(np.sqrt(sum(np.square(translated_pixel_pos)))))
        x, y = translated_pixel_pos
        angle = int(np.arctan2(y, x) / np.pi * 32767)  # Might be other way around

        ball_info_v[:] = angle, distance, img_x, img_y, r
        return
    
    # Goal proc
    def goal_proc_setup(self):
        self.goal_bounds_v = Array(c_uint8, (140, 200, 80, 180, 255, 150, 35, 120, 50, 45, 255, 255))  # 2 bgoal HSV bounds, ygoal HSV bounds
        # I just removed ke (constant of 3 at the end of the array), hopefully nothing breaks
        self.bgoal_info_v = Array(c_int16, (0, 0, 0, 0, 0, 0, 0, 0, 0))  # center angle, left angle, right angle, distance, x, y, w, h, rect_angle
        self.ygoal_info_v = Array(c_int16, (0, 0, 0, 0, 0, 0, 0, 0, 0))
        self.enabled_goals_v = Value(c_uint8, 3)  # 2^0 bit: Blue goal enabled, 2^1 bit: Yellow goal enabled

        self.broadcaster.register_proc(
            "Goals",
            self.goal_proc_init,
            self.goal_proc_loop,
            None,
            (
                self.goal_bounds_v,
                self.bgoal_info_v,
                self.ygoal_info_v,
                self.enabled_goals_v,
                self.camera.frame_shape
            )
        )

    @staticmethod
    def goal_proc_init(goal_bounds_v, bgoal_info_v, ygoal_info_v, enabled_goals_v, frame_shape):
        # hsv_frame = np.zeros(shape=frame_shape, dtype=np.uint8)  # DEBUG
        goal_mask_frame = np.zeros(shape=frame_shape[:2], dtype=np.uint8)
        values = [goal_bounds_v, bgoal_info_v, ygoal_info_v, enabled_goals_v]

        return [goal_mask_frame, values]  # Also add hsv_frame if using for debug
    
    @staticmethod
    def goal_proc_loop(base_args: BaseProcArgs, keep_args: dict):
        enabled = base_args.enabled
        if not enabled:
            return
        
        frame_size, frame_shape, latest_idx, latest_timestamp, frame = base_args[:5]
        goal_mask_frame, values = keep_args  # also unpack hsv_frame from here if using
        goal_bounds_v, bgoal_info_v, ygoal_info_v, enabled_goals_v = values
        
        enabled_goals = enabled_goals_v.value
        # cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, hsv_frame)  # DEBUG (Replace line below)
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, frame)
        cfg = np.array(goal_bounds_v, dtype=np.uint8)

        # Do once for blue goal, do once for yellow goal
        for enabled_flag, lbound, ubound, goal_info_v in ((1, cfg[0:3], cfg[3:6], bgoal_info_v), (2, cfg[6:9], cfg[9:12], ygoal_info_v)):
            if not enabled_goals & enabled_flag:  # Flag for enabling that color goal (1 blue, 2 yellow)
                goal_info_v[:] = (0, 0, 0, 0, 0, 0, 0, 0, 0)
                continue

            cv2.inRange(frame, lbound, ubound, goal_mask_frame)  # Replace with hsv_frame if using
            goalContours = cv2.findContours(goal_mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if not goalContours:
                goal_info_v[:] = (0, 0, 0, 0, 0, 0, 0, 0, 0)
                continue

            bestContour = max(goalContours, key=cv2.contourArea)
            rect = cv2.minAreaRect(bestContour)

            rect_points = cv2.boxPoints(rect)  # Gives corners as [(x1, y1), (x2, y2)...]
            rect_point_angles = list(np.arctan2(rect_points[:, 1], rect_points[:, 0]))
            leftest, rightest = enclosing_interval(rect_point_angles)
            rect_point_angles.remove(leftest)
            rect_point_angles.remove(rightest)
            left_angle, right_angle = rect_point_angles  # Non min/max points


            (goal_center_x, goal_center_y), (goal_width, goal_height), rect_angle = rect

            # polygon = cv2.approxPolyDP(bestContour, ke * cv2.arcLength(bestContour, True), True)
            # goal_center_x, goal_center_y = np.mean(polygon[:, 0, :], axis=0).astype(np.int16)

            center_x = frame_shape[1] // 2 + OFFSET_X
            center_y = frame_shape[0] // 2 + OFFSET_Y
            relative_x = goal_center_x - center_x
            relative_y = goal_center_y - center_y
            distance = min(32767, int(np.hypot(relative_x, relative_y)))
            angle = int(np.arctan2(relative_y, relative_x) / np.pi * 32767)
            left_angle = int(left_angle / np.pi * 32767)
            right_angle = int(right_angle / np.pi * 32767)
            goal_center_x = int(goal_center_x)
            goal_center_y = int(goal_center_y)
            goal_width = int(goal_width)
            goal_height = int(goal_height)
            rect_angle = int(rect_angle)

            goal_info_v[:] = angle, left_angle, right_angle, distance, goal_center_x, goal_center_y, goal_width, goal_height, rect_angle
            # time.sleep(0.5)  # DEBUG
            # if enabled_flag == 1:  # DEBUG
                # cv2.imwrite("/var/www/html/frame.jpg", np.hstack((cv2.cvtColor(goal_mask_frame, cv2.COLOR_GRAY2BGR), frame, hsv_frame)))  # DEBUG
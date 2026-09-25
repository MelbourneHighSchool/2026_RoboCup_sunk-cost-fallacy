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

def distance_regression(dist_px):
    return -13562.2348/(dist_px - 361.39354) - 35.70338

def get_internal_angles(angles):
    angles = sorted(angles)
    gaps = [
        angles[i+1] - angles[i]
        for i in range(3)
    ]
    gaps.append(angles[0] + 360 - angles[3])
    i = np.argmax(gaps)
    ordered = angles[i+1:] + angles[:i+1]
    inner = ordered[1:3]
    return inner

class Vision:
    def __init__(self):
        self.camera = Camera()
        self.broadcaster = Broadcaster(self.camera.create_broadcaster_args())

        self.center_v = self.camera.v_center
        self.ball_proc_setup()
        self.goal_proc_setup()

    @property
    def ball_info(self):
        angle, distance, x, y, r = self.ball_info_v[:]
        angle = self.convert_directional_angle(angle)
        return (angle, distance, x, y, r)

    @property
    def ygoal_info(self):
        angle, ang_width, distance, x, y, w, h, rang  = self.ygoal_info_v[:]
        angle = self.convert_directional_angle(angle)
        ang_width = self.convert_quantitative_angle(ang_width)
        return (angle, ang_width, distance, x, y, w, h, rang)

    @property
    def bgoal_info(self):
        angle, ang_width, distance, x, y, w, h, rang = self.bgoal_info_v[:]
        angle = self.convert_directional_angle(angle)
        ang_width = self.convert_quantitative_angle(ang_width)
        return (angle, ang_width, distance, x, y, w, h, rang)
    
    def goal_localise_pos(self, heading):
        if heading is None:
            return None
        angle, distance, indated = self.goal_localise_info_v[:]
        angle = np.radians(self.convert_directional_angle(angle) + heading)
        return (distance * np.array((np.sin(angle), np.cos(angle))), indated)

    @staticmethod
    def convert_directional_angle(angle):
        return -(angle * 180 / 32767 + 90) % 360

    @staticmethod
    def convert_quantitative_angle(angle):
        return angle * 180 / 32767

    def load_config(self, config):
        center = config.get_value("center")
        if center is not None:
            self.center_v[:] = center
        
        hsv = config.get_value("hsv")
        if hsv is not None:
            self.ball_bounds_v[:] = hsv["ball"]["low"] + hsv["ball"]["high"]
            self.goal_bounds_v[:] = hsv["bgoal"]["low"] + hsv["bgoal"]["high"] + hsv["ygoal"]["low"] + hsv["ygoal"]["high"]
    
    def load_config_beta(self, config):
        center = config.get_value("center")
        if center is not None:
            self.center_v[:] = center
        
        hsv = config.get_value("hsv")
        if hsv is not None:
            blh = round(hsv["ball"]["low"][0] / 360 * 255)
            bls = round(hsv["ball"]["low"][1] / 100 * 255)
            blv = round(hsv["ball"]["low"][2] / 100 * 255)
            buh = round(hsv["ball"]["high"][0] / 360 * 255)
            bus = round(hsv["ball"]["high"][1] / 100 * 255)
            buv = round(hsv["ball"]["high"][2] / 100 * 255)
            self.ball_bounds_v[:] = (blh, bls, blv, buh, bus, buv)

            bglh = round(hsv["bgoal"]["low"][0] / 360 * 255)
            bgls = round(hsv["bgoal"]["low"][1] / 100 * 255)
            bglv = round(hsv["bgoal"]["low"][2] / 100 * 255)
            bguh = round(hsv["bgoal"]["high"][0] / 360 * 255)
            bgus = round(hsv["bgoal"]["high"][1] / 100 * 255)
            bguv = round(hsv["bgoal"]["high"][2] / 100 * 255)
            yglh = round(hsv["ygoal"]["low"][0] / 360 * 255)
            ygls = round(hsv["ygoal"]["low"][1] / 100 * 255)
            yglv = round(hsv["ygoal"]["low"][2] / 100 * 255)
            yguh = round(hsv["ygoal"]["high"][0] / 360 * 255)
            ygus = round(hsv["ygoal"]["high"][1] / 100 * 255)
            yguv = round(hsv["ygoal"]["high"][2] / 100 * 255)
            
            self.goal_bounds_v[:] = (bglh, bgls, bglv, bguh, bgus, bguv, yglh, ygls, yglv, yguh, ygus, yguv)
    
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
        self.ball_time_v = Array(c_float, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0))  # last 10 timestamps of ball detection

        self.broadcaster.register_proc(
            "Ball",
            self.ball_proc_init,
            self.ball_proc_loop,
            None,
            (
                self.ball_bounds_v,
                self.ball_info_v,
                self.camera.frame_shape,
                self.ball_time_v
            )
        )
    
    @staticmethod
    def ball_proc_init(ball_bounds_v, ball_info_v, frame_shape, ball_time_v):
        mask_frame = np.zeros(shape=frame_shape[:2], dtype=np.uint8)  # 2D array since only 1 channel
        return [ball_bounds_v, ball_info_v, mask_frame, ball_time_v]

    @staticmethod
    def ball_proc_loop(base_args: BaseProcArgs, keep_args: list):
        enabled = base_args.enabled
        if not enabled:
            return
        
        frame_size, frame_shape, center, latest_idx, latest_timestamp, frame = base_args[:6]
        ball_bounds_v, ball_info_v, mask_frame, ball_time_v = keep_args

        ball_time_v[:] = ball_time_v[1:] + [latest_timestamp]  # Shift left and add new timestamp

        # print(ball_time_v[:])  # DEBUG

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
        
        center = np.array(center, dtype=np.int16)
        img_x, img_y = pixel_pos
        translated_pixel_pos = pixel_pos - center
        distance = min(32767, int(np.sqrt(sum(np.square(translated_pixel_pos)))))
        x, y = translated_pixel_pos
        angle = int(np.arctan2(y, x) / np.pi * 32767)  # Might be other way around

        ball_info_v[:] = angle, distance, img_x, img_y, r
        return
    
    # Goal proc
    def goal_proc_setup(self):
        self.goal_bounds_v = Array(c_uint8, (130, 150, 150, 150, 255, 255, 35, 120, 50, 45, 255, 255))  # 2 bgoal HSV bounds, ygoal HSV bounds
        # I just removed ke (constant of 3 at the end of the array), hopefully nothing breaks
        self.bgoal_info_v = Array(c_int16, (0, 0, 0, 0, 0, 0, 0, 0))  # center angle, left angle, right angle, distance, x, y, w, h, rect_angle
        self.ygoal_info_v = Array(c_int16, (0, 0, 0, 0, 0, 0, 0, 0))
        self.goal_localise_info_v = Array(c_int16, (0, 0, 0))
        self.enabled_goals_v = Value(c_uint8, 3)  # 2^0 bit: Blue goal enabled, 2^1 bit: Yellow goal enabled

        # self.goal_time_v = Array(c_float, (0, 0, 0, 0, 0, 0, 0, 0, 0, 0))  # last 10 timestamps of goal detection

        self.broadcaster.register_proc(
            "Goals",
            self.goal_proc_init,
            self.goal_proc_loop,
            None,
            (
                self.goal_bounds_v,
                self.bgoal_info_v,
                self.ygoal_info_v,
                self.goal_localise_info_v,
                self.enabled_goals_v,
                self.camera.frame_shape,
                # self.goal_time_v
            )
        )

    @staticmethod
    def goal_proc_init(goal_bounds_v, bgoal_info_v, ygoal_info_v, goal_localise_info_v, enabled_goals_v, frame_shape):
        # hsv_frame = np.zeros(shape=frame_shape, dtype=np.uint8)  # DEBUG
        goal_mask_frame = np.zeros(shape=frame_shape[:2], dtype=np.uint8)
        values = [goal_bounds_v, bgoal_info_v, ygoal_info_v, goal_localise_info_v, enabled_goals_v]

        return [goal_mask_frame, values]  # Also add hsv_frame if using for debug
    
    @staticmethod
    def goal_proc_loop(base_args: BaseProcArgs, keep_args: dict):
        enabled = base_args.enabled
        if not enabled:
            return
        
        frame_size, frame_shape, center, latest_idx, latest_timestamp, frame = base_args[:6]
        goal_mask_frame, values = keep_args  # also unpack hsv_frame from here if using
        goal_bounds_v, bgoal_info_v, ygoal_info_v, goal_localise_info_v, enabled_goals_v = values
        
        enabled_goals = enabled_goals_v.value
        # cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, hsv_frame)  # DEBUG (Replace line below)
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, frame)
        cfg = np.array(goal_bounds_v, dtype=np.uint8)

        goal_vectors = [None, None]  # For localisation, requires both goals to be enabled (0 blue, 1 yellow)

        # Do once for blue goal, do once for yellow goal
        for enabled_flag, lbound, ubound, goal_info_v in ((1, cfg[0:3], cfg[3:6], bgoal_info_v), (2, cfg[6:9], cfg[9:12], ygoal_info_v)):
            if not enabled_goals & enabled_flag:  # Flag for enabling that color goal (1 blue, 2 yellow)
                goal_info_v[:] = (0, 0, 0, 0, 0, 0, 0, 0)
                continue

            cv2.inRange(frame, lbound, ubound, goal_mask_frame)  # Replace with hsv_frame if using
            goalContours = cv2.findContours(goal_mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            if not goalContours:
                goal_info_v[:] = (0, 0, 0, 0, 0, 0, 0, 0)
                continue

            bestContour = max(goalContours, key=cv2.contourArea)
            rect = cv2.minAreaRect(bestContour)

            (goal_center_x, goal_center_y), (goal_width, goal_height), rect_angle = rect

            center_x = center[0]
            center_y = center[1]
            relative_x = goal_center_x - center_x
            relative_y = goal_center_y - center_y

            distance_px = np.hypot(relative_x, relative_y)
            if distance_px > 0.01:  # Put in goal vector in real distance
                distance_real = distance_regression(distance_px)
                goal_vectors[enabled_flag >> 1] = (relative_x, relative_y) * distance_real / distance_px
            else:
                goal_vectors[enabled_flag >> 1] = None

            rect_points = cv2.boxPoints(rect)  # Gives corners as [(x1, y1), (x2, y2)...]
            rect_point_angles = np.arctan2(rect_points[:, 1] - center_y, rect_points[:, 0] - center_x)
            internal_angles = get_internal_angles(rect_point_angles)
            center_angle = sum(internal_angles) / 2  # But what if angle wrapping? Might have to add 180°
            ang_width = abs(internal_angles[1] - internal_angles[0]) / 2
            if abs(internal_angles[1] - internal_angles[0]) >= np.pi:
                center_angle += np.pi
                ang_width = np.pi - ang_width

            # Convert angle to ±180
            center_angle %= (2 * np.pi)
            if center_angle > np.pi:
                center_angle -= 2 * np.pi

            angle = int(center_angle / np.pi * 32767)
            ang_width = int(ang_width / np.pi * 32767)

            # angular_goal_width = min(np.abs(rect_point_angles - angle))

            # polygon = cv2.approxPolyDP(bestContour, ke * cv2.arcLength(bestContour, True), True)
            # goal_center_x, goal_center_y = np.mean(polygon[:, 0, :], axis=0).astype(np.int16)

            goal_center_x = int(goal_center_x)
            goal_center_y = int(goal_center_y)
            goal_distance = min(32767, int(distance_px))
            goal_width = int(goal_width)
            goal_height = int(goal_height)
            rect_angle = int(rect_angle)

            goal_info_v[:] = angle, ang_width, goal_distance, goal_center_x, goal_center_y, goal_width, goal_height, rect_angle

        # After everything, *localise.*
        if None not in goal_vectors:
            field_center_relative_x, field_center_relative_y = -(goal_vectors[0] + goal_vectors[1]) / 2
            direction = np.arctan2(field_center_relative_y, field_center_relative_x)
            direction = int(direction / np.pi * 32767)
            distance = np.hypot(field_center_relative_x, field_center_relative_y)
            distance = int(min(32767, distance))
            goal_localise_info_v[:] = direction, distance, True
        else:
            goal_localise_info_v[2] = False

            # time.sleep(0.5)  # DEBUG
            # if enabled_flag == 1:  # DEBUG
                # cv2.imwrite("/var/www/html/frame.jpg", np.hstack((cv2.cvtColor(goal_mask_frame, cv2.COLOR_GRAY2BGR), frame, hsv_frame)))  # DEBUG
    
    # def bot_proc_setup(self):
    #     self.field_bounds_v = Array(c_uint8, (70, 51, 77, 110, 255, 255))
    #     # self.line_bounds_v = Array(c_uint8, (0, 0, 230, 255, 20, 255))
    #     self.bot_info_v = Array(c_int16, (0, 1, 2, 3))  # Angle, distance, x, y for only one robot
    #     self.broadcaster.register_proc(
    #         "Bot",
    #         self.goal_proc_init,
    #         self.goal_proc_loop,
    #         None,
    #         (
    #             self.field_bounds_v,
    #             # self.line_bounds_v,
    #             self.bot_info_v,
    #             self.camera.frame_shape
    #         )
    #     )

    # @staticmethod
    # def bot_proc_init(self, field_bounds_v, bot_info_v, frame_shape):
    #     mask_frame = np.zeros(shape=frame_shape[:2], dtype=np.uint8)
    #     return [field_bounds_v, bot_info_v, mask_frame]

    # @staticmethod
    # def bot_proc_loop(self, base_args, keep_args):
    #     enabled = base_args.enabled
    #     if not enabled:
    #         return
        
    #     frame_size, frame_shape, center, latest_idx, latest_timestamp, frame = base_args[:6]
    #     field_bounds_v, bot_info_v, mask_frame = keep_args

    #     cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, frame)
    #     cv2.inRange(frame, field_bounds_v[:3], field_bounds_v[3:], mask_frame)  # Test it is possible to inRange like this
    #     cv2.morphologyEx(frame, )
    #     contours = cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    #     if not contours:
    #         print("[Bot Detection] WARN: Cannot find field contour. Mask wrong colour perhaps?")
    #         return
        
    #     bestContour = max(contours, key=cv2.contourArea)
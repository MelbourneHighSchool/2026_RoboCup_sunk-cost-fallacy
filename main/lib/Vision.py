"""
TO DO
- Create shared variables to store the config and stuff
- Create the ball process
- Make enemy/teammate/wall/goal shit + process
- Add a way to calibrate mirror centre...

- Possibly add locks for shared variables (but not sure if necessary)
"""

from lib.localise import findCentre
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

fieldW, fieldL = 152, 213  # Note: These values subtract 30cm because they're for lines, not field
maximumError = 5
def findCenter(pcloud) -> np.array:
    rayAngles = pcloud[:, 0]
    rayDists = pcloud[:, 1]
    rayXs, rayYs = np.cos(rayAngles) * rayDists, np.sin(rayAngles) * rayDists

    maxX, minX, maxY, minY = max(rayXs), min(rayXs), max(rayYs), min(rayYs)
    closestWall = np.argmin(((maxX-rayXs), (rayXs - minX), (maxY-rayYs),(rayYs-minY),rayDists.astype(float) * maximumError),0)
    rightWallPs, leftWallPs, topWallPs, bottomWallPs = rayXs[closestWall==0], rayXs[closestWall==1], rayYs[closestWall==2], rayYs[closestWall==3]

    cxR, numRightWall = np.average(rightWallPs) - (fieldL / 2), len(rightWallPs)
    cxL, numLeftWall = np.average(leftWallPs) + (fieldL / 2), len(leftWallPs)   
    # print(cxL, numLeftWall, cxR, numRightWall)
    # print(max(leftWallPs), min(leftWallPs))
    cx = 0
    if abs(cxL - cxR) > fieldL * maximumError:
        if numRightWall > numLeftWall:
            cx = cxR
        else:
            cx = cxL
    else:
        cx = (cxR * numRightWall + cxL * numLeftWall) / (numLeftWall + numRightWall) 

    cyT, numTopWall = np.average(topWallPs) - (fieldW / 2), len(topWallPs)
    cyB, numBottomWall = np.average(bottomWallPs) + (fieldW / 2), len(bottomWallPs)
    cy = 0
    if abs(cyB - cyT) > fieldW * maximumError:
        if numTopWall > numBottomWall:
            cy = cyT
        else:
            cy = cyB
    else:
        cy = (cyT * numTopWall + cyB * numBottomWall) / (numBottomWall + numTopWall)
        
    return np.array((cx, cy))

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

def parse_goal_contour(goal_contour, img_center):
    rect = cv2.minAreaRect(goal_contour)

    goal_center, goal_dims, rect_angle = rect

    relative_x = goal_center[0] - img_center[0]
    relative_y = goal_center[1] - img_center[1]
    distance_px = np.hypot(relative_x, relative_y)

    rect_points = cv2.boxPoints(rect)  # Gives corners as [(x1, y1), (x2, y2)...]
    rect_point_angles = np.arctan2(rect_points[:, 1] - img_center[1], rect_points[:, 0] - img_center[0])
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

    return (angle, ang_width, goal_distance, goal_center_x, goal_center_y, goal_width, goal_height, rect_angle)

def find_goals(mask, img_center):
    goal_contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    if not goal_contours:
        return (0, 0, 0, 0, 0, 0, 0, 0)

    sorted_contours = sorted(goal_contours, key=cv2.contourArea, reverse=True)
    big_contour = sorted_contours[0]
    lil_contour = sorted_contours[1]
    big_info = parse_goal_contour(big_contour, img_center)
    lil_info = parse_goal_contour(lil_contour, img_center)

    # Consider the little contour
    # Area ratio between lil : big must be >0.8
    if cv2.contourArea(lil_contour) / cv2.contourArea(big_contour) > 0.8:
        return big_info

    # If lil open angle too small then fahh nah
    if lil_info[1] / big_info[1] < 0.2:
        return big_info

    # Otherwise, get the goal whose nearest post is closest to the goals 
    lal = lil_info[0] - lil_info[1]
    lar = lil_info[0] + lil_info[1]
    bal = big_info[0] - big_info[1]
    bar = big_info[0] + big_info[1]

    lil_score = -np.min(np.abs(lal), np.abs(lar)) * np.sign(lal * lar)  # Multiply by sign makes score +ve if forward is inside goal posts
    big_score = -np.min(np.abs(bal), np.abs(bar)) * np.sign(bal * bar)
    if big_score >= lil_score:
        return big_info
    else:
        return lil_info

class Vision:
    def __init__(self):
        self.camera = Camera()
        self.broadcaster = Broadcaster(self.camera.create_broadcaster_args())

        self.yaw_v = Value(c_int16, 0)
        self.center_v = self.camera.v_center
        self.ball_proc_setup()
        self.goal_proc_setup()
        # self.lines_proc_setup()

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
    
    def yaw_callback(self, yaw):
        self.yaw_v.value = int(yaw * 32767 / 180)
    
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
                self.enabled_goals_v,
                self.camera.frame_shape,
                # self.goal_time_v
            )
        )

    @staticmethod
    def goal_proc_init(goal_bounds_v, bgoal_info_v, ygoal_info_v, enabled_goals_v, frame_shape):
        # hsv_frame = np.zeros(shape=frame_shape, dtype=np.uint8)  # DEBUG
        goal_mask_frame = np.zeros(shape=frame_shape[:2], dtype=np.uint8)
        values = [goal_bounds_v, bgoal_info_v, ygoal_info_v, enabled_goals_v]

        return [goal_mask_frame, values]  # Also add hsv_frame if using for debug
    
    @staticmethod
    def goal_proc_loop(base_args: BaseProcArgs, keep_args: list):
        enabled = base_args.enabled
        if not enabled:
            return
        
        frame_size, frame_shape, center, latest_idx, latest_timestamp, frame = base_args[:6]
        goal_mask_frame, values = keep_args  # also unpack hsv_frame from here if using
        goal_bounds_v, bgoal_info_v, ygoal_info_v, enabled_goals_v = values
        
        enabled_goals = enabled_goals_v.value
        # cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, hsv_frame)  # DEBUG (Replace line below)
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, frame)
        cfg = np.array(goal_bounds_v, dtype=np.uint8)

        # Do once for blue goal, do once for yellow goal
        for enabled_flag, lbound, ubound, goal_info_v in ((1, cfg[0:3], cfg[3:6], bgoal_info_v), (2, cfg[6:9], cfg[9:12], ygoal_info_v)):
            if not enabled_goals & enabled_flag:  # Flag for enabling that color goal (1 blue, 2 yellow)
                goal_info_v[:] = (0, 0, 0, 0, 0, 0, 0, 0)
                continue

            cv2.inRange(frame, lbound, ubound, goal_mask_frame)  # Replace with hsv_frame if using
            goal_info_v[:] = find_goals(goal_mask_frame, center)

            # time.sleep(0.5)  # DEBUG
            # if enabled_flag == 1:  # DEBUG
                # cv2.imwrite("/var/www/html/frame.jpg", np.hstack((cv2.cvtColor(goal_mask_frame, cv2.COLOR_GRAY2BGR), frame, hsv_frame)))  # DEBUG
    
    def lines_proc_setup(self):
        self.line_bounds_v = Array(c_uint8, (0, 0, 180, 255, 30, 255))
        self.estimated_pos_v = Array(c_int16, (0, 0))  # x, y
        self.dbg_points = Array(c_float, 720)

        self.broadcaster.register_proc(
            "Lines",
            self.lines_proc_init,
            self.lines_proc_loop,
            None,
            (
                self.line_bounds_v,
                self.estimated_pos_v,
                self.camera.frame_shape,
                self.yaw_v,
                self.dbg_points
            )
        )
    
    @staticmethod
    def lines_proc_init(line_bounds_v, estimated_pos_v, frame_shape, yaw_v, dbg_points):
        lines_mask_frame = np.zeros(shape=frame_shape[:2], dtype=np.uint8)
        return [lines_mask_frame, line_bounds_v, estimated_pos_v, yaw_v, dbg_points]
    
    @staticmethod
    def lines_proc_loop(base_args: BaseProcArgs, keep_args: list):
        enabled = base_args.enabled
        if not enabled:
            return
        
        frame_size, frame_shape, center, latest_idx, latest_timestamp, frame = base_args[:6]
        lines_mask_frame, line_bounds_v, estimated_pos_v, yaw_v, dbg_points = keep_args

        cv2.blur(frame, (9, 9))
        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL, frame)
        cv2.inRange(frame, np.array(line_bounds_v[:3]), np.array(line_bounds_v[3:]), lines_mask_frame)
        cv2.circle(lines_mask_frame, center, 52, 0, cv2.FILLED)

        max_radius = int(np.hypot(frame_shape[0], frame_shape[1]) * 0.7)  # Rough estimate
        polar_img = cv2.warpPolar(
            lines_mask_frame,
            dsize=(max_radius, 360), # 360 angle steps, max_radius distance resolution
            center=center,
            maxRadius=max_radius,
            flags=cv2.WARP_POLAR_LINEAR + cv2.INTER_NEAREST
        )

        pcloud = []
        has_hits = polar_img > 0
        first_hit_radii = np.argmax(has_hits, axis=1)
        yaw = yaw_v.value * np.pi / 32767
        lines_contour = np.zeros(shape=(360, 1, 2), dtype=np.int32)
        for i in range(360):
            if first_hit_radii[i] <= 0:
                dbg_points[2*i] = 0
                dbg_points[2*i+1] = 0
                continue
            angle = i*2*np.pi/360
            pcloud.append((angle - yaw, distance_regression(first_hit_radii[i])))
            dbg_points[2*i] = first_hit_radii[i] * np.cos(pcloud[-1][0])
            dbg_points[2*i+1] = first_hit_radii[i] * np.sin(pcloud[-1][1])
            lines_contour[i][0] = np.array((first_hit_radii[i] * np.cos(angle), first_hit_radii[i] * np.sin(angle)), dtype=np.int32)

        if not pcloud:
            estimated_pos_v[:] = (-32767, -32767)
            return
        field_center = findCentre(np.array(pcloud)).astype(int)
        # print(field_center)
        estimated_pos_v[:] = field_center
    
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
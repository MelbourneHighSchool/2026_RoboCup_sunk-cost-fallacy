from lib.config import Config
from lib.imu import IMU
from lib.Vision import Vision
from lib.drive import Drive
from lib.interface import WSServer
from lib.kicker import Kicker
from lib.dribbler import Dribbler
from lib.localize import Localizer
import time
import cv2
import numpy as np
import math
import board
from enum import Enum

SEND_FRAME = True

SOLENOID_PIN = board.D21
PULSE_S = 0.02

# -----------------------------------------------------------------------------------------------------------
class RobotRegions(Enum):
    NONE = 0
    GOAL_SIDE = 1
    MIDDLE = 2
    MIDDLE_SIDE = 3
    OWN_GOAL = 4
    OWN_GOAL_SIDE = 5

class PossessionStates(Enum):
    HEADING_TO_GOAL = 0
    ALIGNING_WITH_GOAL = 1
    BALL_HIDING = 2

class Robot:
    def __init__(self, drive=None, imu=None, config=None):
        self.config = config if config is not None else Config()
        self.imu = imu if imu is not None else IMU()
        self.imu.calibrate_yaw()

        if drive is not None:
            self.drive = drive
        else:
            self.drive = Drive.from_config(self.config)

        self.kicker = Kicker(SOLENOID_PIN, PULSE_S)

        self.dribbler = Dribbler(self.config)

        # Ball tracking
        self.see_ball = False
        self.have_ball = False
        self.ball_dir = None
        self.ball_dist = None
        self.last_ball_dir = None
        self.last_ball_dist = None
        self.last_ball_see_time = 0

        # Movement
        self.move_spd = 0
        self.move_dir = 0
        self.rot_spd = 0

        # Position & Orientation
        self.bot_dir = 0
        self.pos_x = 0
        self.pos_y = -450
        self._yaw_error = 0
        self._yaw_error_time = time.monotonic()

        # Constants
        self.GIVE_UP_CHASING_BALL_TIME = 1.0
        #TODO
        self.loc = Localizer(["array","of","tofs"],self.imu)
    
    def main_loop(self):
        self.update_stuff()
        if self.ball_dir is not None:
            self.yaw_correct(self.to_absolute_dir(self.ball_dir))
        else:
            self.rot_spd = 0
        if self.ball_dir is not None and self.ball_dist is not None:
            # self.ball_capture()
            pass
        else:
            self.move_spd = 0
            self.move_dir = 0
        self.move()

        # print(self.ball_dir, self.ball_dist)
    
    def ball_capture(self):
        MOVE_FORWARD_ANGLE = 60  # ±
        ORBIT_RADIUS = 60 # Pixels
        SPD_MAX = 0.2
        SPD_MID = 0.1
        SPD_MIN = 0.03

        

        self.move_spd = 0.1
        # if abs(self.ball_dir) < 10:
        #     self.move_spd = SPD_MAX
        # elif self.ball_dist > 200:
        #     self.move_spd = self.clamp((SPD_MAX - SPD_MID)/100 * (self.ball_dist - 200) + SPD_MID, SPD_MID, SPD_MAX)
        # elif self.ball_dist < ORBIT_RADIUS + 10:
        #     self.move_spd = self.clamp((SPD_MAX - SPD_MID)/MOVE_FORWARD_ANGLE * abs(self.ball_dir) + SPD_MID, SPD_MID, SPD_MAX)
        # else:
        #     self.move_spd = SPD_MID

        if abs(self.ball_dir) < MOVE_FORWARD_ANGLE:
            print("Forward")
            # self.move_spd = self.sigmoid(abs(self.ball_dir), SPD_MIN, SPD_MAX, 0.1, MOVE_FORWARD_ANGLE / 2)
            # self.move_spd = (abs(self.ball_dir) - 30)**2 / 6000 + SPD_MIN
            self.move_dir = self.ball_dir * 2.2   

        elif self.ball_dist <= ORBIT_RADIUS:
            print("Too close")
            distance_ratio = (ORBIT_RADIUS - self.ball_dist) / ORBIT_RADIUS
            orbit_angle = 90 + distance_ratio * 90
            self.move_dir = self.ball_dir + np.copysign(orbit_angle, self.ball_dir)
        
        else:
            print("Regular orbit")
            self.move_dir = self.ball_dir + np.copysign(math.degrees(np.asin(ORBIT_RADIUS/self.ball_dist)), self.ball_dir)

    
    def update_ball_info(self, ball_dir, ball_dist):
        self.see_ball = ball_dist != 0.0

        if self.see_ball:
            self.ball_dir = self.wrap_angle(ball_dir)
            self.ball_dist = ball_dist

            self.last_ball_dir = self.ball_dir
            self.last_ball_dist = self.ball_dist
            
            self.last_ball_see_time = time.monotonic()
        elif time.monotonic() - self.last_ball_see_time < self.GIVE_UP_CHASING_BALL_TIME:
            self.ball_dir = self.last_ball_dir
            self.ball_dist = self.last_ball_dist
        else:
            self.ball_dir = None
            self.ball_dist = None

    def update_goal_info(self): # TODO
         pass        

    def update_stuff(self):
        # Region
        if abs(self.pos_x) > 350:
            if self.pos_y > 700:
                self.region = RobotRegions.GOAL_SIDE
            elif self.pos_y < -640:
                self.region = RobotRegions.OWN_GOAL_SIDE
            elif abs(self.pos_x):
                self.region = RobotRegions.MIDDLE_SIDE
        elif self.pos_y < -640:
            self.region = RobotRegions.OWN_GOAL
        elif self.pos_y < 1100:
            self.region = RobotRegions.MIDDLE
        else:
            self.region = RobotRegions.NONE

        # Yaw
        self.bot_dir = -1 * self.wrap_angle(self.imu.get_yaw())
        

    # ----- Actions ----- #
    def move(self):
        # self.avoid_out_of_bounds()
        self.drive.move(self.move_dir, self.move_spd, self.rot_spd)
        # print(self.move_dir, self.move_spd, self.rot_spd)


    def avoid_out_of_bounds(self):
        """Adjust move direction and speed for the x and y components to avoid going out of bounds"""
        BOUND_LINE_X = 635 + 10     # mm, ±
        BOUND_LINE_Y = 940 + 10     # mm, ±
        START_SLOWDOWN_X_DIST = 100
        START_SLOWDOWN_Y_DIST = 100
        AVOID_WALL_SPD = 0.1

        if self.move_dir is None or self.move_spd is None:
            return

        x_dir = 1 if self.pos_x >= 0 else -1
        y_dir = 1 if self.pos_y >= 0 else -1

        move_dir_absolute = self.to_absolute_dir(self.move_dir)
        move_vec_x = self.move_spd * math.sin(math.radians(move_dir_absolute))
        move_vec_y = self.move_spd * math.cos(math.radians(move_dir_absolute))

        if abs(self.pos_x) > BOUND_LINE_X:
            move_vec_x = -AVOID_WALL_SPD * x_dir
        if abs(self.pos_y) > BOUND_LINE_Y:
            move_vec_y = -AVOID_WALL_SPD * y_dir

        if abs(self.pos_x) > BOUND_LINE_X - START_SLOWDOWN_X_DIST and np.sign(move_vec_x) == x_dir:
            distance_to_wall = BOUND_LINE_X - abs(self.pos_x)
            scale = self.clamp(distance_to_wall / START_SLOWDOWN_X_DIST, 0.0, 1.0)
            move_vec_x *= scale

        if abs(self.pos_y) > BOUND_LINE_Y - START_SLOWDOWN_Y_DIST and np.sign(move_vec_y) == y_dir:
            distance_to_wall = BOUND_LINE_Y - abs(self.pos_y)
            scale = self.clamp(distance_to_wall / START_SLOWDOWN_Y_DIST, 0.0, 1.0)
            move_vec_y *= scale

        final_speed = math.hypot(move_vec_x, move_vec_y)
        if final_speed == 0:
            self.move_spd = 0
            self.move_dir = 0
            return

        final_angle = math.degrees(math.atan2(move_vec_x, move_vec_y))
        self.move_dir = self.to_relative_dir(self.wrap_angle(final_angle))
        self.move_spd = final_speed

    def kick(self):
        self.kicker.kick()

    def dribble(self):
        # self.dribbler.set_speed(-0.5)
        pass
    
    def stop_dribbler(self):
        self.dribbler.set_speed(0)

    def yaw_correct(self, target_angle=0, speed=0.3, kp=0.001, kd=0.00001):
        """PD Yaw correction"""
        if self.bot_dir is None:
            self.rot_spd = 0
            return

        error = self.wrap_angle(target_angle - self.bot_dir)

        if abs(error) < 2:
            self.rot_spd = 0
            self._yaw_error = error
            self._yaw_error_time = time.monotonic()
            return self.rot_spd

        now = time.monotonic()
        dt = now - self._yaw_error_time
        derivative = (error - self._yaw_error) / dt if dt > 0 else 0
        derivative = self.clamp(derivative, -100, 100)
        correction = kp * error + kd * derivative
        self.rot_spd = self.clamp(correction, -abs(speed), abs(speed))
        self._yaw_error = error
        self._yaw_error_time = now

        print(dt, error, derivative)
    # ----- Helper functions ----- #
    def wrap_angle(self, theta):
            """Returns same angle but in [-180°,180°)"""
            if theta is None:
                return None
            return (theta + 180) % 360 - 180

    def sigmoid(self, value, min=0, max=1, steepness=1, centre=0):
            # https://www.desmos.com/calculator/jkqwos4tzh
            a = math.exp(steepness * (centre - value))
            return (max - min) * (1 / (1 + a)) + min

    def to_absolute_dir(self, relative_dir):
            """Input a direction relative to the bot orientation\nReturns a direction that ignores bot orientation"""
            if relative_dir is None:
                return None
            return relative_dir + self.bot_dir
    
    def to_relative_dir(self, absolute_dir):
        """Input a direction that ignores bot orientation\nReturns a direction relative to the bot orientation"""
        if absolute_dir is None:
            return None
        return absolute_dir - self.bot_dir

    def clamp(self, value, mn, mx):
            return max(mn, min(value, mx))

    def real_dist(self, pixel_dist):
        """Input: Distance from centre of camera in pixels
        Output: pproximate real distance in mm"""
        # https://www.desmos.com/calculator/p7hn4afpw8

        if pixel_dist is None or pixel_dist == 0.0:
            return pixel_dist
        real_dist_cm = 10**((pixel_dist + 75)/165)
        return real_dist_cm * 10

# -----------------------------------------------------------------------------------------------------------

# Main script
server = WSServer()
server.run()

vision = Vision()
vision.start()
R = min(vision.camera.size) / 2

# Create robot instance
robot = Robot()

drive = robot.drive

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
    
            # Show ball
            frame = cv2.circle(frame, (int(bx), int(by)), br, (0, 50, 150), 8, cv2.LINE_AA)
            frame = cv2.putText(frame, f"Ball angle: {bangle}", (padx, pady), *default_font)
            frame = cv2.putText(frame, f"Ball distance: {bdist}", (padx, pady+line_spacing), *default_font)

            # Send frame
            server.send_frame(frame)
        
        # Update robot stuff
        robot.update_ball_info(bangle, bdist)
        # print(bangle, bdist)
        # robbot.update_goal_info
        robot.main_loop()
        
    except KeyboardInterrupt:
        break

drive.stop()
vision.deinit()
robot.dribbler.set_speed(0)
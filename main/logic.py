SEND_FRAME = True

from lib.config import Config
from lib.imu import IMU
from lib.Vision import Vision
from lib.drive import Drive
from lib.interface import WSServer

import time
import cv2
import numpy as np
import math

# -----------------------------------------------------------------------------------------------------------

class Robot:
    
    def __init__(self, drive):
        self.drive = drive
        
        # Ball tracking
        self.see_ball = False
        self.ball_dir = None
        self.ball_dist = None
        self.last_ball_dir = None
        self.last_ball_dist = None
        self.last_ball_see_time = 0
        
        # Movement
        self.move_spd = 0
        self.move_dir = 0

        # Position & Orientation
        self.bot_dir = 0
        
        # Constants
        self.GIVE_UP_CHASING_BALL_TIME = 0.6

    
    def update_movement(self):
        self.ball_capture()
    
    def ball_capture(self):
        """Ball capture behavior - orbits ball and moves towards it"""
        MOVE_FORWARD_ANGLE = 20  # ±
        ORBIT_RADIUS = 370 # Pixels
        SPD_MAX = 0.2
        SPD_MIN = 0.03

        self.move_spd = self.sigmoid(self.ball_dist, SPD_MIN, SPD_MAX, 0.002, 700)

        # If ball is roughly forward, go towards it
        if abs(self.ball_dir) < MOVE_FORWARD_ANGLE:
            self.move_dir = self.ball_dir * 1.5
            modified_radius = self.sigmoid(abs(self.ball_dir), 0, ORBIT_RADIUS, 0.5, MOVE_FORWARD_ANGLE / 2)
            self.move_dir = self.ball_dir + np.copysign(math.degrees(np.asin(modified_radius / self.ball_dist)), self.ball_dir)
        else:           
            # If too close to ball, go away from it
            if self.ball_dist < ORBIT_RADIUS:
                distance_ratio = (ORBIT_RADIUS - self.ball_dist) / ORBIT_RADIUS
                orbit_angle = 90 + distance_ratio * 90
                self.move_dir = self.ball_dir + np.copysign(orbit_angle, self.ball_dir)
            # Else move in an angle that is tangent to a circle centered at the ball
            else:  
                self.move_dir = self.ball_dir + np.copysign(math.degrees(np.asin(ORBIT_RADIUS / self.ball_dist)), self.ball_dir)
    
    def update_ball_info(self, ball_dir, ball_dist):
        """Update ball information and tracking"""
        self.see_ball = ball_dist != 0 or ball_dir is not None or ball_dist is not None

        if self.see_ball:
            self.last_ball_dir = self.ball_dir
            self.last_ball_dist = self.ball_dist
            self.last_ball_see_time = time.monotonic()

        if self.see_ball:
            self.ball_dir = ball_dir
            self.ball_dist = ball_dist
        elif time.monotonic() - self.last_ball_see_time < self.GIVE_UP_CHASING_BALL_TIME:
            self.ball_dir = self.last_ball_dir
            self.ball_dist = self.last_ball_dist
        
        self.ball_dir = self.wrap_angle(ball_dir)
        self.ball_dist = ball_dist

    def update_goal_info(self):
         pass        


    # ----- Basic actions ----- #
    def move(self):
        """Execute movement based on move_dir and move_spd"""
        self.drive.move(angle=self.move_dir, speed=self.move_spd)

    def kick(self):
        # TODO: Actually kick
        pass

    def dribble(self):
        # TODO: Actually dribble
        pass
    
    def stop_dribbler(self):
        # TODO
        pass

    # ----- Helper functions ----- #
    def wrap_angle(self, theta):
            """Returns same angle but in [-180°,180°)"""
            if theta is None:
                return None
            return (theta + 180) % 360 - 180

    def sigmoid(self, value, min=0, max=1, steepness=1, centre=0):
            #https://www.desmos.com/calculator/pdsx583kvo
            
            a = math.exp(steepness * (value - centre))
            return (max - min) * (a / (1 + a)) + min

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

    def clamp(self, value, min, max):
            return max(min, min(value, max))

# -----------------------------------------------------------------------------------------------------------

# Main script
server = WSServer()
server.run()

vision = Vision()
vision.start()
R = min(vision.camera.size) / 2

imu = IMU()
config = Config()

drive = Drive(imu, config)

# Create Robot instance
robot = Robot(drive)

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
            frame = cv2.circle(frame, (int(bx+R), int(by+R)), br, (0, 50, 150), 8, cv2.LINE_AA)
            frame = cv2.putText(frame, f"Ball angle: {bangle}", (padx, pady), *default_font)
            frame = cv2.putText(frame, f"Ball distance: {bdist}", (padx, pady+line_spacing), *default_font)
    
            # Send frame
            server.send_frame(frame)
        
        # Update robot stuff
        robot.update_ball_info(bangle, bdist)
        robot.update_movement()
        robot.move()
        
    except KeyboardInterrupt:
        break

drive.stop()
vision.deinit()

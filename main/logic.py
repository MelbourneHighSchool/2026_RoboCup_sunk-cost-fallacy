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


# Robot Class Definition
class Robot:
    """Robot class with ball_capture behavior"""
    
    def __init__(self, drive):
        """Initialize Robot with required attributes"""
        self.drive = drive
        
        # Ball tracking
        self.see_ball = False
        self.ball_dir = None
        self.ball_dist = None
        self.last_ball_dir = None
        self.last_ball_dist = None
        self.last_ball_see_time = time.monotonic()
        
        # Movement
        self.move_spd = 0
        self.move_dir = 0
        
        # Constants
        self.GIVE_UP_CHASING_BALL_TIME = 0.6
    
    def dribble(self):
        """Enable dribbler (placeholder for now)"""
        # print("Dribbler ON")
        pass
    
    def stop_dribbler(self):
        """Disable dribbler (placeholder for now)"""
        # print("Dribbler OFF")
        pass
    
    def sigmoid(self, value, min=0, max=1, steepness=1, centre=0):
        """Sigmoid function for smooth speed scaling
        https://www.desmos.com/calculator/pdsx583kvo
        """
        a = math.exp(steepness * (value - centre))
        return (max - min) * (a / (1 + a)) + min
    
    def ball_capture(self):
        # print(self.ball_dir)
        """Ball capture behavior - orbits ball and moves towards it"""
        MOVE_FORWARD_ANGLE = 20  # ±
        ORBIT_RADIUS = 370
        SPD_MAX = 0.2
        SPD_MIN = 0.03

        if self.see_ball:
            ball_dir = self.ball_dir
            ball_dist = self.ball_dist
        else:
            # For when last_ball_see_time is less than GIVE_UP_CHASING_BALL_TIME seconds ago
            ball_dir = self.last_ball_dir
            ball_dist = self.last_ball_dist

        # https://www.desmos.com/calculator/lhvwffvnag
        self.move_spd = self.sigmoid(ball_dist, SPD_MIN, SPD_MAX, 0.002, 700)
        # self.move_spd = 0.1

        # If ball is roughly forward, go towards it
        if abs(ball_dir) < MOVE_FORWARD_ANGLE:
            print("FORWARD")
            self.dribble()
            self.move_dir = ball_dir * 1.5

            modified_radius = self.sigmoid(abs(ball_dir), 0, ORBIT_RADIUS, 0.5, MOVE_FORWARD_ANGLE / 2)
            self.move_dir = ball_dir + np.copysign(math.degrees(np.asin(modified_radius / ball_dist)), ball_dir)
        else:
            self.stop_dribbler()
            
            # If too close to ball, go away from it
            if ball_dist < ORBIT_RADIUS:
                print("too close")
                distance_ratio = (ORBIT_RADIUS - ball_dist) / ORBIT_RADIUS
                orbit_angle = 90 + distance_ratio * 90
                self.move_dir = ball_dir + np.copysign(orbit_angle, ball_dir)
            # Else move in an angle that is tangent to a circle centered at the ball
            else:  
                print("Orbit")
                self.move_dir = ball_dir + np.copysign(math.degrees(np.asin(ORBIT_RADIUS / ball_dist)), ball_dir)
    
    def update_ball_info(self, ball_dir, ball_dist):
        """Update ball information and tracking"""
        if ball_dir is not None and ball_dist is not None:
            self.last_ball_dir = self.ball_dir
            self.last_ball_dist = self.ball_dist
            self.last_ball_see_time = time.monotonic()
        
        self.ball_dir = self.wrap_angle(ball_dir)
        self.ball_dist = ball_dist
        self.see_ball = self.ball_dist != 0 or self.ball_dir is not None or self.ball_dist is not None
    
    def move(self):
        """Execute movement based on move_dir and move_spd"""
        self.drive.move(angle=self.move_dir, speed=self.move_spd)
        # print(f"Moving: dir={self.move_dir:.1f}°, spd={self.move_spd:.2f}")
    

    def wrap_angle(self, theta):
            """Returns same angle but in [-180°,180°)"""
            if theta is None:
                return None
            return (theta + 180) % 360 - 180


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
        
        # Update robot with ball info
        robot.update_ball_info(bangle, bdist)
        
        # Execute ball capture behavior
        robot.ball_capture()
        
        # Move robot based on behavior
        robot.move()
        
        # print(bangle)
    except KeyboardInterrupt:
        break

drive.stop()
vision.deinit()

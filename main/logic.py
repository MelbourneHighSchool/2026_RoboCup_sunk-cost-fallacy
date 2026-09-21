from lib.config import Config
from lib.imu import IMU
from lib.Vision import Vision
from lib.drive import Drive
from lib.interface import WSServer
from lib.kicker import Kicker
from lib.dribbler import Dribbler
from lib.localize import Localizer
from lib.tof import ToF
from lib.switch import Switch
from lib.PDcontroller import PDController
from lib.break_beam import BreakBeam

import time
import cv2
import numpy as np
import math
import board
from enum import Enum
from collections import deque

SOLENOID_PIN = board.D21
PULSE_S = 0.02

# -----------------------------------------------------------------------------------------------------------
class AttackStates(Enum):
    # Unused atm
    DEFAULT = 0
    BALL_HIDING = 1
    PULL_BALL_FROM_GOAL_SIDE = 2
    PULL_BALL_FROM_OWN_GOAL_SIDE = 3
    CATCH_BALL_ON_SIDE = 4

class RecentValues:
    def __init__(self, num=12, min_samples=3):
        self.values = deque(maxlen=num)
        self.min_samples = min_samples

    def update(self, new_value):
        self.values.append(new_value)

    def check(self, condition, true_threshold):
        if len(self.values) < self.min_samples:
            return False  # not enough data yet

        percent_true = sum(condition(v) for v in self.values) / len(self.values)
        return percent_true >= true_threshold
    
# -----------------------------------------------------------------------------------------------------------

class Robot:
    def __init__(self, drive=None, imu=None, config=None):

        self.config = config if config is not None else Config()
        self.imu = imu if imu is not None else IMU()
        self.imu.calibrate_yaw()
        self.drive = drive if drive is not None else Drive.from_config(self.config)

        self.kicker = Kicker(SOLENOID_PIN, PULSE_S)
        self.dribbler = Dribbler(self.config)
        self.tofs = (ToF(0x50), ToF(0x51), ToF(0x52), ToF(0x53), ToF(0x54), ToF(0x55), ToF(0x56), ToF(0x5f))
        self.breakbeam = BreakBeam(board.D14)
        # Switches
        switches = self.config.get_value("switches")

        goal_conf = switches.get("goal")
        self.goal_switch = Switch(goal_conf["pin"], goal_conf["on_high"])

        run_conf = switches.get("run")
        self.run_switch = Switch(str(run_conf["pin"]), run_conf["on_high"])

        # State machine
        self.state = AttackStates.DEFAULT

        # Flags
        self.ENABLE_BALL_HIDING = False

        # Ball
        self.see_ball = False
        self.ball_dir = None
        self.ball_dist = None
        self.last_ball_dir = None
        self.last_ball_dist = None
        self.last_ball_see_time = 0
        self.BALL_DIR_TIMEOUT = 0.8

        self.have_ball = False
        self.last_possession_time = None
        self.POSSESSION_TIMEOUT = 0.2

        # Goal
        self.target_goal_is_blue = False
        self.see_goal = False
        self.goal_dir = None
        self.goal_dist = None
        self.see_own_goal = False
        self.own_goal_dir = None
        self.own_goal_dist = None

        # Movement
        self.move_spd = 0
        self.move_dir = 0
        self.rot_spd = 0

        # Position & Orientation
        self.bot_dir = 0
        self.loc = Localizer(self.tofs, self.imu)
        self.pos_x = 0
        self.pos_y = -450
        self.yaw_error = 0
        self.yaw_error_time = time.monotonic()
        self.yaw_controller = PDController(kp=0.001, kd=0.00001, max_derivative=100, debug=False)
        self.yaw_controller.previous_error = self.yaw_error
        self.yaw_controller.previous_time = self.yaw_error_time

        self.loc_x_record = RecentValues(num=12, min_values=3)
        self.loc_y_record = RecentValues(num=12, min_values=3)
        self.is_at_goal_side = False
        self.is_at_own_goal_side = False
        self.is_at_middle_side = False
        
    # ----- Main ----- #

    def main_loop(self):
        self.update_stuff()

        self.attack_loop()
        # self.defence_loop()

        if self.is_ready_to_shoot():
            self.kick() # Kick if there is a line of shot regardless of role

        self.move()

        # DEBUG
        # print(self.ball_dir, self.ball_dist, self.have_ball)
        # print(self.goal_dir, self.goal_ang_width, self.goal_dist)
        # print(self.own_goal_dir, self.own_goal_dist)
        # print(self.region)
        # print(self.pos_x, self.pos_y)
    
    # ----- Offence ----- #

    def attack_loop(self):
        if self.have_ball:
            self.possession_behaviour()
        elif self.see_ball:
            # Trying to gain possession of ball
            if self.is_at_middle_side:
                self.yaw_correct()
                self.ball_capture(soft=True)
            elif self.is_at_own_goal_side:
                self.yaw_correct_relative(self.ball_dir)
                self.ball_capture(soft=True)
            elif self.is_at_goal_side:
                self.yaw_correct_relative(self.ball_dir)
                self.ball_capture(soft=True)
            else:
                self.yaw_correct_towards_goal()
                self.ball_capture()
        else:
            # If no ball seen, move towards (0, -150)
            self.stop_dribbler()
            self.yaw_correct()
            self.move_dir = self.to_relative_dir(self.angle_towards(0, -150))
    
    def ball_capture(self, soft=False):
        MOVE_FORWARD_ANGLE = 30  # ±
        ORBIT_RADIUS = 67
        SPD_MAX = 0.29
        SPD_MIN = 0.02

        self.move_spd = self.sigmoid(self.ball_dist, SPD_MIN, SPD_MAX, 0.028, 71)

        if abs(self.ball_dir) < 10:
            self.dribble()
            self.move_dir = self.ball_dir
            self.move_spd = SPD_MAX if soft == False else self.lerp(self.clamp(self.ball_dist, 80, 140), 80, 140, 0.03, SPD_MAX)
            return
        elif abs(self.ball_dir) < MOVE_FORWARD_ANGLE:
            self.dribble()
            self.move_dir = self.ball_dir * 3
        elif self.ball_dist <= ORBIT_RADIUS:
            self.stop_dribbler()
            distance_ratio = (ORBIT_RADIUS - self.ball_dist) / ORBIT_RADIUS
            orbit_angle = 90 + distance_ratio * 90
            self.move_dir = self.ball_dir + np.copysign(orbit_angle, self.ball_dir)
        else:
            self.stop_dribbler()
            self.move_dir = self.ball_dir + np.copysign(math.degrees(np.asin(ORBIT_RADIUS/self.ball_dist)), self.ball_dir)

    def possession_behaviour(self):
        self.dribble()

        if self.see_goal and not self.goal_side:
            self.move_spd = 0
            self.rotate_towards_goal()
        else:
            self.move_spd = 0
            self.rot_spd = 0

    def ball_hide(self, yaw=90):

        if self.pos_y > 500:
            if self.see_goal:
                self.rotate_towards_goal()
                self.move_spd = 0
                return
            else:
                self.move_spd = 0.03
                self.move_dir = self.to_relative_dir(self.angle_towards(0, 500))
        else:
            self.yaw_correct(np.sign(self.pos_x) * yaw, speed=0.1, max_spd=0.03)

        if yaw - 10 < abs(self.bot_dir) < yaw + 10:
            if abs(self.pos_x) < 450:
                self.move_dir = self.to_relative_dir(90 * np.sign(self.pos_x))
                self.move_spd = 0.03
            else:
                self.move_dir = self.to_relative_dir(0)
                self.move_spd = 0.03
        else:
            self.move_spd = 0

    def is_ready_to_shoot(self):
        return self.have_ball and self.see_goal and abs(self.goal_dir) < 0.7 * self.goal_ang_width

    # ----- Defence -----#

    def defence_loop(self):
        KEEP_DIST = 50
        TOLERANCE = 5
        GOAL_WEIGHT = 0.1
        BALL_WEIGHT = 1.0

        if self.see_ball and self.ball_dist < 85:
            self.attack_loop()
            return
        
        if not self.see_own_goal:
            self.rot_dir = 0
            self.move_dir = self.to_relative_dir(self.angle_towards(0, -400))

        if self.see_ball and abs(self.to_absolute_dir(self.own_goal_dir)) < 90 and abs(self.to_absolute_dir(self.ball_dir)) > 90:
            return

        if KEEP_DIST - TOLERANCE < self.own_goal_dist < KEEP_DIST + TOLERANCE:
            move_dir_goal = 0
        elif self.own_goal_dist > KEEP_DIST + TOLERANCE:
            move_dir_goal = self.own_goal_dir
        else:
            move_dir_goal = self.own_goal_dir + 180

        intercept = 0
        if self.see_ball:
            abs_goal_dir = self.to_absolute_dir(self.own_goal_dir)
            abs_ball_dir = self.to_absolute_dir(self.ball_dir)
            diff = self.wrap_angle(abs_goal_dir - abs_ball_dir)
            move_dir_ball = self.to_absolute_dir(np.sign(diff) * 90)

            # Face the circular midpoint between the ball and the direction
            # opposite the goal, keeping the two objects on opposite sides
            target_x = math.sin(math.radians(abs_ball_dir)) - math.sin(math.radians(abs_goal_dir))
            target_y = math.cos(math.radians(abs_ball_dir)) - math.cos(math.radians(abs_goal_dir))
            angle_target = math.degrees(math.atan2(target_x, target_y))
            self.yaw_correct(self.wrap_angle(angle_target), tolerance=12)

            intercept = diff
        else:
            self.yaw_correct(self.to_absolute_dir(self.own_goal_dir) + 180, tolerance=12)
            if move_dir_goal == 0:
                self.move_spd = 0
            else:
                self.move_spd = 0.03
                self.move_dir = move_dir_goal
            return

        goal_angle = math.radians(self.to_absolute_dir(move_dir_goal))
        ball_angle = math.radians(move_dir_ball)
        move_vec_x = GOAL_WEIGHT * math.sin(goal_angle) + BALL_WEIGHT * math.sin(ball_angle)
        move_vec_y = GOAL_WEIGHT * math.cos(goal_angle) + BALL_WEIGHT * math.cos(ball_angle)

        self.move_spd = min(0.3, math.hypot(move_vec_x, move_vec_y))
        if self.move_spd == 0:
            self.move_dir = 0
            return

        if self.see_ball:
            self.move_spd *= ((180 - abs(intercept)) / 180)**0.5
        move_angle = math.degrees(math.atan2(move_vec_x, move_vec_y))
        self.move_dir = self.to_relative_dir(self.wrap_angle(move_angle))

    # ----- Updates ----- #  

    def update_stuff(self):
        # IMU
        self.bot_dir = self.wrap_angle(self.imu.get_yaw())

        # Localisation
        self.pos_x, self.pos_y = self.loc.getPosition()
        self.loc_x_record.update(self.pos_x)
        self.loc_y_record.update(self.pos_y)

        is_on_side = self.loc_x_record.check(lambda x: abs(x) > 350, 0.8) # True if at least 80% of recent values are say that the bot is on the side of the field
        is_at_goal_y = self.loc_y_record.check(lambda y: y > 650, 0.8)
        is_at_own_goal_y = self.loc_y_record.check(lambda y: y < -650, 0.8)

        self.is_at_goal_side = is_on_side and is_at_goal_y
        self.is_at_own_goal_side = is_on_side and is_at_own_goal_y
        self.is_at_middle_side = is_on_side and not is_at_goal_y and not is_at_own_goal_y

    def update_ball_info(self, ball_dir, ball_dist):
            self.see_ball = True
    
            if ball_dist != 0.0:
                self.ball_dir = self.wrap_angle(ball_dir)
                self.ball_dist = ball_dist
    
                self.last_ball_dir = self.ball_dir
                self.last_ball_dist = self.ball_dist
                
                self.last_ball_see_time = time.monotonic()
            elif time.monotonic() - self.last_ball_see_time < self.BALL_DIR_TIMEOUT:
                self.ball_dir = self.last_ball_dir
                self.ball_dist = self.last_ball_dist
            else:
                self.see_ball = False
                self.ball_dir = None
                self.ball_dist = None
    
            if self.see_ball:
                self.ball_pos_x = self.ball_dist * math.sin(math.radians(self.ball_dir))
                self.ball_pos_y = self.ball_dist * math.cos(math.radians(self.ball_dir))
    
            now = time.monotonic()
            if self.breakbeam.read():
                self.last_possession_time = now
    
            self.have_ball = (self.last_possession_time is not None and now - self.last_possession_time < self.POSSESSION_TIMEOUT)

    def update_goal_info(self, bgoal_angle, bgoal_ang_width, bgoal_dist, ygoal_angle, ygoal_ang_width, ygoal_dist):
        if self.target_goal_is_blue:
            target_angle, target_dist = bgoal_angle, bgoal_dist
            own_angle, own_dist = ygoal_angle, ygoal_dist
            goal_ang_width = bgoal_ang_width
            own_goal_ang_width = ygoal_ang_width
        else:
            target_angle, target_dist = ygoal_angle, ygoal_dist
            own_angle, own_dist = bgoal_angle, bgoal_dist
            goal_ang_width = ygoal_ang_width
            own_goal_ang_width = bgoal_ang_width

        self.goal_dir = self.wrap_angle(target_angle) if target_dist != 0 else None
        self.goal_ang_width = self.wrap_angle(goal_ang_width) if target_dist != 0 else None
        self.goal_dist = self.approx_real_dist(target_dist) / 10 if target_dist != 0 else None
        self.see_goal = self.goal_dir is not None and self.goal_dist is not None
        self.own_goal_dir = self.wrap_angle(own_angle) if own_dist != 0 else None
        self.own_goal_dist = self.approx_real_dist(own_dist) / 10 if own_dist != 0 else None
        self.own_goal_ang_width = self.wrap_angle(own_goal_ang_width) if target_dist != 0 else None
        self.see_own_goal = self.own_goal_dir is not None and self.own_goal_dist is not None

    # ----- Actions ----- #

    def move(self):
        self.avoid_out_of_bounds()
        self.drive.move(self.move_dir, self.move_spd, self.rot_spd)

        # DEBUG
        # self.drive.move(0, 0, self.rot_spd)

    def avoid_out_of_bounds(self):
        BOUND_LINE_X = 550    # mm, ±
        BOUND_LINE_Y = 750     # mm, ±
        START_SLOWDOWN_X_DIST = 100
        START_SLOWDOWN_Y_DIST = 100
        AVOID_WALL_SPD = 0.02

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
        self.dribbler.set_speed(-1)
    
    def stop_dribbler(self):
        self.dribbler.set_speed(0)

    def yaw_correct(self, target_angle=0.0, max_spd=0.3, speed=1.0, kp=0.002, kd=0.00005, tolerance=2):
        """PD Yaw correction"""
        if self.bot_dir is None:
            self.rot_spd = 0
            return

        error = self.wrap_angle(target_angle - self.bot_dir)

        self.yaw_controller.kp = kp
        self.yaw_controller.kd = kd
        self.yaw_controller.max_derivative = 100

        correction = self.yaw_controller.compute(0.0, -error)

        if abs(error) < tolerance:
            self.rot_spd = 0
        else:
            self.rot_spd = self.clamp(speed * correction, -abs(max_spd), abs(max_spd))

        self.yaw_error = error
        self.yaw_error_time = self.yaw_controller.previous_time

    def yaw_correct_relative(self, relative_angle, max_spd=0.3, speed=1.0, kp=0.002, kd=0.00005, tolerance=2):
        error = self.wrap_angle(relative_angle)

        self.yaw_controller.kp = kp
        self.yaw_controller.kd = kd
        self.yaw_controller.max_derivative = 100

        correction = self.yaw_controller.compute(0.0, -error)

        if abs(error) < tolerance:
            self.rot_spd = 0
        else:
            self.rot_spd = self.clamp(speed * correction, -abs(max_spd), abs(max_spd))

    def rotate_about_dribbler(self, speed=0.05):
        """Input: sign(speed) = 1 for clockwise, 1 for anticlockwise"""
        self.rot_spd = speed
        self.move_dir = np.sign(speed) * -90
        self.move_spd = 1 * abs(speed) # 1 happened to be the ratio that works

    def rotate_towards_goal(self):
        if self.see_goal:
            self.rotate_about_dribbler(np.sign(self.goal_dir), 0.025)
            # self.rot_spd = 0.01 * np.sign(self.goal_dir)
        else:
            print("rotate_towards_goal is being called when goal is not visible")

    def yaw_correct_towards_goal(self):
        if self.see_goal:
            self.yaw_correct_relative(np.sign(self.goal_dir) * (abs(self.goal_dir))**1.32)
        else:
            print("yaw_correct_towards_goal is being called when goal is not visible")
    
    def yaw_correct_line_of_shot(self):
        if self.see_goal and self.see_ball:
            self.goal_pos_x = self.goal_dist * math.sin(math.radians(self.goal_dir))
            self.goal_pos_y = self.goal_dist * math.cos(math.radians(self.goal_dir))

            yaw_vec_x = self.goal_pos_x - self.ball_pos_x
            yaw_vec_y = self.goal_pos_y - self.ball_pos_y
            yaw = math.degrees(math.atan2(yaw_vec_x, yaw_vec_y))
            self.yaw_correct_relative(yaw * 1)
        else:
            print("yaw_correct_line_of_shot is being called when goal or ball is not visible")

    # ----- Helper functions ----- #

    @staticmethod
    def wrap_angle(theta):
            """Returns same angle but in [-180°,180°)"""
            if theta is None:
                return None
            return (theta + 180) % 360 - 180

    @staticmethod
    def sigmoid(value, min=0, max=1, steepness=0.1, centre=0):
            # https://www.desmos.com/calculator/jkqwos4tzh
            a = math.exp(steepness * (centre - value))
            return (max - min) * (1 / (1 + a)) + min

    def to_absolute_dir(self, relative_dir):
            """Input a direction relative to the bot orientation\nReturns a direction that ignores bot orientation"""
            if relative_dir is None:
                return None
            return self.wrap_angle(relative_dir + self.bot_dir)
    
    def to_relative_dir(self, absolute_dir):
        """Input a direction that ignores bot orientation\nReturns a direction relative to the bot orientation"""
        if absolute_dir is None:
            return None
        return self.wrap_angle(absolute_dir - self.bot_dir)

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min_value, min(value, max_value))

    @staticmethod
    def approx_real_dist(pixel_dist):
        """
        Input: Distance from centre of camera in pixels
        Output: approximate real distance in mm
        
        PLEASE DON'T CHANGE THIS FUNCTION BECAUSE MANY CONSTANTS ARE BASED ON THIS
        """
        # https://www.desmos.com/calculator/gkbgcxzhoo

        if pixel_dist is None or pixel_dist == 0.0:
            return None
        approx_real_dist_cm = 10**((pixel_dist + 75)/165)
        return approx_real_dist_cm * 10 + 105 # 210/2 = 105

    def lerp(self, value, input_min, input_max, output_min, output_max):
        return self.clamp(output_min + (value - input_min) * (output_max - output_min) / (input_max - input_min), output_min, output_max)

    def angle_towards(self, obj_x, obj_y):
        """Returns the ABSOLUTE direction towards a coordinate on a field."""
        return self.wrap_angle(math.degrees(math.atan2(obj_x - self.pos_x, obj_y - self.pos_y)))   

# -----------------------------------------------------------------------------------------------------------

# Main script
SEND_FRAME = False

# Create robot instance

robot = Robot()

server = WSServer()
if SEND_FRAME:
    server.run()

vision = Vision()
vision.load_config(robot.config)
vision.start()
R = min(vision.camera.size) / 2

# Font settings
default_font = (cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 5, cv2.LINE_AA)
padx = 15
pady = 25
line_spacing = 30

robot.target_goal_is_blue = robot.goal_switch.read()
print("===\nTarget: " + ("Blue goal" if robot.target_goal_is_blue else "Yellow goal") + "\n===")

paused = False

while True:
    try:
        if not robot.run_switch.read():
            robot.move_spd = 0
            robot.move_dir = 0
            robot.rot_spd = 0
            robot.drive.move(robot.move_dir, robot.move_spd, robot.rot_spd)
            paused = True
            continue
        if paused and robot.run_switch.read():
            # unpause
            paused = False
            robot.imu.yaw_offset = robot.imu.get_yaw()
            robot.target_goal_is_blue = robot.goal_switch.read()
            print("===\nTarget: " + ("Blue goal" if robot.target_goal_is_blue else "Yellow goal") + "\n===")
        # Ball

        bangle, bdist, bx, by, br = vision.ball_info


        # Blue goal
        (
            bgoal_angle,
            bgoal_ang_width,
            bgoal_dist,
            bgoal_x,
            bgoal_y,
            bgoal_width,
            bgoal_height,
            bgoal_rect_angle,
        ) = vision.bgoal_info

        # Yellow goal
        (
            ygoal_angle,
            ygoal_ang_width,
            ygoal_dist,
            ygoal_x,
            ygoal_y,
            ygoal_width,
            ygoal_height,
            ygoal_rect_angle,
        ) = vision.ygoal_info

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
        robot.update_goal_info(
            bgoal_angle,
            bgoal_ang_width,
            bgoal_dist,
            ygoal_angle,
            ygoal_ang_width,
            ygoal_dist
        )

        robot.main_loop()
    except KeyboardInterrupt:
        break

vision.deinit()
robot.drive.stop()
robot.stop_dribbler()
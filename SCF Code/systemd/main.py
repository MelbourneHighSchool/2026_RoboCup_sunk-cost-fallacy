import numpy as np
import math
import time
from enum import Enum
from lib.dribbler import Dribbler
from lib.drive import Drive
# from lib.camera import Cameras
from lib.break_beam import BreakBeam
import board

from lib.kicker import Kicker
from lib.config import Config
from lib.imu import IMU
# from lib.localisation import Localisation
from lib.switch import Switch
from lib.tof import ToF

USE_COMM_MODULE = True
USE_COMMUNICATION = False

SOLENOID_PIN = board.D27
COMM_MODULE_PIN = board.D18
PULSE_S = 0.02

# ----- MODES/STATES ----- #
class RobotState(Enum):
    NO_SEE_BALL = 0
    CHASING_BALL = 1
    HAVE_BALL = 2

class PossessionState(Enum):
    HEADING_TO_GOAL = 0
    BALL_HIDING = 1

class GoalColour(Enum):
    BLUE = "Blue"
    YELLOW = "Yellow"

class RobotMode(Enum):
    PENALTY = 0
    OFFENCE = 1
    DEFENCE = 2

# ----- Main logic ----- #
class Robot():

    def __init__(self):
        """Initialise"""
        # Initialize Hardware Interfaces
        self.imu = IMU()
        self.config = Config()
        self.drive = Drive(self.imu, self.config)
        # self.cameras = Cameras()

        self.dribbler = Dribbler(self.config)
        self.break_beam = BreakBeam(board.D17)
        self.pause_switch = Switch(board.D16, self.config)
        self.was_paused = True
        self.kicker = Kicker(SOLENOID_PIN, PULSE_S)
        self.goal_switch = Switch(board.D12, self.config)

        # Variables
        ## States
        self.state : RobotState = RobotState.NONE
        self.possession_state : PossessionState = PossessionState.NONE
        self.see_ball = False
        self.have_ball = False
        self.last_ball_dir = None
        self.last_ball_dist = None
        self.last_ball_see_time = 0
        self.see_goal = False
        self.see_own_goal = False
        self.yaw_correct_on = True

        ## Time
        self.last_time = time.monotonic()
        self.dt = 0.01

        ## Movement
        self.move_spd = 0  # 0-1 normalised speed
        self.move_dir = 0
        self.target_yaw = None  # desired heading, or None to disable yaw correction

        ## Position & Orientation data
        self.bot_dir = 0  # IMU

        ## Ball
        self.ball_dir = None
        self.ball_dist = None

        ## Goal
        self.target_goal = GoalColour.YELLOW if self.goal_switch.read() else GoalColour.BLUE
        self.goal_dir = None
        self.goal_dist = None
        self.own_goal_dir = None
        self.own_goal_dist = None


        # CONSTANTS
        self.DRIBBLER_ROT_SPD = 1

        self.YAW_CORRECT_SPD = 0.2
 
        self.READY_TO_SHOOT_ANGLE = 10

        ## Ball capturing
        self.GIVE_UP_CHASING_BALL_TIME = 0.6 # seconds
        self.BALL_ORBIT_RADIUS = 14
        self.BALL_CHASE_SPD_MAX = 0.2
        self.BALL_CHASE_SPD_MIN = 0.03


    def on_update(self):
        """Main Loop"""
        # dt
        current_time = time.monotonic()
        self.dt = current_time - self.last_time
        self.last_time = current_time

        # Orientation
        self.bot_dir = self.drive.yaw

        # Camera / perception is optional until the camera stack exists.
        if self.cameras is not None:
            self.cameras.process()

        # Ball
        if self.ball_dir is not None and self.ball_dist is not None:
            self.last_ball_dir = self.ball_dir
            self.last_ball_dist = self.ball_dist
            self.last_ball_see_time = time.monotonic()


        self.ball_dir = self.wrap_angle(self.cameras.get_ball_dir())
        self.ball_dist = self.cameras.get_ball_dist()
        self.see_ball = self.ball_dir is not None and self.ball_dist is not None

        self.have_ball = self.break_beam.read()


        # Goal
        if self.target_goal == GoalColour.BLUE:
            self.goal_dir = self.wrap_angle(self.cameras.get_blue_goal_dir())
            self.goal_dist = self.cameras.get_blue_goal_dist()
            self.own_goal_dir = self.wrap_angle(self.cameras.get_yellow_goal_dir())
            self.own_goal_dist = self.cameras.get_yellow_goal_dist()

        elif self.target_goal == GoalColour.YELLOW:
            self.goal_dir = self.wrap_angle(self.cameras.get_yellow_goal_dir())
            self.goal_dist = self.cameras.get_yellow_goal_dist()
            self.own_goal_dir = self.wrap_angle(self.cameras.get_blue_goal_dir())
            self.own_goal_dist = self.cameras.get_blue_goal_dist()

        self.see_goal = self.goal_dir is not None and self.goal_dist is not None
        self.see_own_goal = self.own_goal_dir is not None and self.own_goal_dist is not None

        if self.see_ball or self.have_ball:
            self.last_ball_see_time = time.monotonic()


        # Switch pausing
        paused_by_switch = not self.pause_switch.read()
        if paused_by_switch:
            self.state = RobotState.NONE
            self.possession_state = PossessionState.NONE
            self.drive.stop()
            self.stop_dribbler()
            return

        if self.was_paused:
            self.was_paused = False
            if self.cameras is not None:
                self.cameras.start_streaming()
            self.target_goal = GoalColour.YELLOW if self.goal_switch.read() else GoalColour.BLUE
            self.drive.recalibrate_yaw()
            print(f"Recalibrated robot; New target goal = {self.target_goal}")

        # Update state machine
        self.execute_behaviour()

        # Execute movement
        self.move()


    # ------ State Machine ------ #
    # General logic
    def execute_behaviour(self):
        self.target_yaw = None

        if self.is_ready_to_shoot():
            self.kick()

        if self.have_ball:
            self.dribble()
            self.move_spd = 0
            if self.see_goal:
                self.target_yaw = self.to_absolute_dir(self.goal_dir)

        elif self.see_ball:
            self.ball_capture()

        elif time.monotonic() - self.last_ball_see_time < self.GIVE_UP_CHASING_BALL_TIME:
            pass

        else:
            self.move_spd = 0

        self.move()
    

    
    def ball_capture(self):
        if self.see_ball:
            ball_dir = self.ball_dir
            ball_dist = self.ball_dist
        else:
            ball_dir = self.last_ball_dir
            ball_dist = self.last_ball_dist

        # https://www.desmos.com/calculator/lhvwffvnag
        # self.move_spd = min(BALL_CHASE_SPD_MAX, max(BALL_CHASE_SPD_MIN, self.ball_dist / 200 + 0.01))
        self.move_spd = self.sigmoid(self.ball_dist, self.BALL_CHASE_SPD_MIN, self.BALL_CHASE_SPD_MAX, 0.15, 30)

        if abs(self.ball_dir) < 40:
            # If ball is roughly forward, go towards it
            self.move_dir = self.ball_dir * 1.5
            self.dribble()

        else:
            self.stop_dribbler()
                
            if ball_dist < self.BALL_ORBIT_RADIUS:
                # If too close to ball, go away from it
                distance_ratio = (self.BALL_ORBIT_RADIUS - ball_dist) / self.BALL_ORBIT_RADIUS
                orbit_angle = 90 + distance_ratio * 90
                self.move_dir = ball_dir + np.copysign(orbit_angle, ball_dir)
            
            else: 
                # Else move in an angle that is tangent to a circle centered at the ball
                self.move_dir = ball_dir + np.copysign(math.degrees(np.asin(self.BALL_ORBIT_RADIUS / ball_dist)), ball_dir)
    
    def is_ready_to_shoot(self):
        return (
            self.have_ball
            and self.see_goal
            and abs(self.goal_dir) < self.READY_TO_SHOOT_ANGLE
            # and self.goal_dist < self.READY_TO_SHOOT_DISTANCE
        )

    # ------ Basic Actions ------ #
    def dribble(self):
        self.dribbler.set_torque(self.DRIBBLER_ROT_SPD)

    def stop_dribbler(self):
        self.dribbler.set_torque(0.0)

    def kick(self):
        self.kicker.kick()

    def move(self):
        self.drive.move(angle=self.move_dir, speed=self.move_spd)

        # Rotation
        if self.yaw_correct_on and self.target_yaw is not None:
            self.drive.set_rotation_target(self.target_yaw, speed=self.YAW_CORRECT_SPD)
        elif not self.yaw_correct_on and self.rot_spd is not None:
            self.drive.set_rotation_rate(self.rot_spd)
        else:
            self.drive.set_rotation_none()

    def dribbler_orbit(self, speed=0.05, orbit_sign=1):
        """Orbit aroud the dribbler so that the yaw changes without moving the ball"""
        self.rot_spd = orbit_sign * speed 
        self.move_dir = 90 * orbit_sign
        self.move_spd = speed
        self.dribble()

    # ------ Utility Functions ------ #

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

    def angle_towards(self, bot_x, bot_y, obj_x, obj_y):
        """Returns angle in degrees"""
        theta = math.degrees(math.atan2(obj_y - bot_y, obj_x - bot_x))
        theta = (theta - 90 + 180) % 360 - 180
        return theta

    def wrap_angle(self, theta):
        """Returns same angle but in [-180°,180°)"""
        if theta is None:
            return None
        return (theta + 180) % 360 - 180

    def sigmoid(self, value, min=0, max=1, steepness=1, centre=0):
        # https://www.desmos.com/calculator/pdsx583kvo
        a = math.exp(steepness * (value - centre))
        return (max - min) * (a / (1 + a)) + min
    
    def clamp(self, value, min, max):
        return max(min, min(value, max))

    
    def print_info(self):
        ball_dir = f"{self.ball_dir:.1f}°" if self.ball_dir is not None else "None"
        ball_dist = f"{self.ball_dist:.0f}" if self.ball_dist is not None else "None"
        goal_dir = f"{self.goal_dir:.1f}°" if self.goal_dir is not None else "None"
        goal_dist = f"{self.goal_dist:.0f}" if self.goal_dist is not None else "None"
        print(f"Ball dir: {ball_dir}, dist: {ball_dist}, Goal dir: {goal_dir}, dist: {goal_dist}")


if __name__ == "__main__":
    bot = Robot()
    try:
        while True:
            bot.on_update()
            # bot.print_info()
            time.sleep(0.01)  # limit update rate

    except KeyboardInterrupt:
        print("Robot stopped")

    except Exception as e:
        print("Unhandled error, stopping robot:", e)
        import traceback
        traceback.print_exc()
    finally:
        bot.drive.stop()
        bot.stop_dribbler()

    
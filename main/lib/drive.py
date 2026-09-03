from lib.motor import Motor
import json
import numpy as np

PRIORITY_PATH = 0
PRIORITY_MOVESPD = 1
PRIORITY_ROTSPD = 2

class Drive:
    def __init__(self, motorNE: Motor, motorSE: Motor, motorSW: Motor, motorNW: Motor):
        self.NE = motorNE
        self.SE = motorSE
        self.SW = motorSW
        self.NW = motorNW
        self.motors = (self.NE, self.SE, self.SW, self.NW)
    
    @staticmethod
    def from_config(config) -> "Drive":
        "Returns a Drivebase from a json file containing information about the motors"
        cfg = config._config  # Load up the dictionary from config (a Config class)
        NE = cfg["motors"]["ne"]
        SE = cfg["motors"]["se"]
        SW = cfg["motors"]["sw"]
        NW = cfg["motors"]["nw"]
        
        motors = []
        for mdata in NE, SE, SW, NW:
            motor = Motor(
                mdata["address"],
                elec_angle_offset=mdata["elec_angle_offset"],
                sin_cos_centre=mdata["sin_cos_centre"]
            )
            motors.append(motor)
        
        return Drive(*motors)

    # Basic move function
    def move(self, move_dir: float, move_spd: float, rot_spd = 0.0):
        "Make the robot move in a certain direction with a certain amount of turning indefinitely"
        move_spd = move_spd
        rot_spd = rot_spd
        move_dir *= -1
        speeds = move_dir + np.array([45, 135, 225, 315], dtype=np.float64)
        speeds = np.sin(np.radians(speeds)) * move_spd

        # Motors spin clockwise (to outside observer) if given +ve set_speed_rps value
        self.NE.set_speed(speeds[0] - rot_spd)
        self.SE.set_speed(speeds[1] - rot_spd)
        self.SW.set_speed(speeds[2] - rot_spd)
        self.NW.set_speed(speeds[3] - rot_spd)
    
    def stop(self):
        "Set all motor speeds to 0"
        for m in self.motors:
            m.set_speed(0)
    
    def __del__(self):
        self.stop()
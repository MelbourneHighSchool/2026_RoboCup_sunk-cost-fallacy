from lib.motor import Motor
from lib.config import Config

class Dribbler:
    def __init__(self, config:Config=None):
        config = config if config is not None else Config()
        dribbler_config = config.get_value("motors").get("dribbler")
        self.motor = Motor(dribbler_config["address"],
                            elec_angle_offset=dribbler_config["elec_angle_offset"],
                            sin_cos_centre=dribbler_config["sin_cos_centre"])

    def set_speed(self, speed):
        self.motor.set_speed(speed)

    def set_torque(self, torque):
        self.motor.set_torque(torque)
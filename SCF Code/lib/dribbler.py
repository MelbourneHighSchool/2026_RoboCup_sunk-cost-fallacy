from lib.motor import Motor
from lib.config import Config

DEFAULT_TORQUE_LIMIT = 65536 * 2

class Dribbler:
    def __init__(self, config:Config=None):
        config = config if config is not None else Config()
        dribbler_config = config.get_value("motors").get("dribbler")
        self.torque_limit = dribbler_config.get("torque_limit", DEFAULT_TORQUE_LIMIT)
        self.motor = Motor(dribbler_config["address"],
                            elec_angle_offset=dribbler_config["elec_angle_offset"],
                            sin_cos_centre=dribbler_config["sin_cos_centre"],
                            current_limit_FOC=abs(self.torque_limit),
                            command_mode=2)

    def set_speed(self, speed):
        self.set_torque(speed)

    def set_torque(self, torque):
        torque = max(-1.0, min(torque, 1.0))
        self.motor.set_torque(int(self.torque_limit * torque))
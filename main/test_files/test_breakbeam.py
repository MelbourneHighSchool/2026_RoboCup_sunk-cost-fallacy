from lib.dribbler import Dribbler
from lib.break_beam import BreakBeam
from lib.config import Config

import time
from board import D14

TORQUE = -0.5
SPEED = -0.2

def main(conf:Config):
    torque_mode = bool(input("Torque mode? (Leave blank for speed): "))

    d = Dribbler(conf)
    b = BreakBeam(D14)
    while True:
        if torque_mode:
            d.set_torque(TORQUE)
        else:
            d.set_speed(SPEED)

        print("has ball!" if b.read() else "no ball")
        time.sleep(0.033)

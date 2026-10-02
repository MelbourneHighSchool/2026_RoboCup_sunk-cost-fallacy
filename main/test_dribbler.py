#!/usr/bin/env python3

import time

from lib.config import Config
from lib.dribbler import Dribbler


def main():
    dribbler = Dribbler(Config())
    print("Started dribbler")
    

    try:
        while True:
            dribbler.set_speed(-0.2)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # stop then coast
        dribbler.set_speed(0)
        dribbler.set_torque(0)
        print("Stopped dribbler")

if __name__ == "__main__":
    main()

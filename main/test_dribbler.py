#!/usr/bin/env python3

import time

from lib.config import Config
from lib.dribbler import Dribbler


def main():
    dribbler = Dribbler(Config())
    print("Started dribbler")
    dribbler.set_speed(-1)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        dribbler.set_speed(0)
        print("Stopped dribbler")

if __name__ == "__main__":
    main()

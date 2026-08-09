import time

from lib.dribbler import Dribbler

if __name__ == "__main__":
    dribbler = Dribbler()
    try:
        dribbler.set_speed(1.0)
        time.sleep(5)
    finally:
        dribbler.set_speed(0)
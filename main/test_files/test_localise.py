from lib.tof import ToF
from lib.imu import IMU
from lib.localize import Localizer
from lib.drive import Drive
import time

import numpy as np

def main(config):
    tofs = (ToF(0x50), ToF(0x51), ToF(0x52), ToF(0x53), ToF(0x54), ToF(0x55), ToF(0x56), ToF(0x5f))
    imu = IMU()

    imu.calibrate_yaw()

    print('Initialising...')

    localiser = Localizer(tofs, imu)

    drive = Drive.from_config(config)

    while True:
        x, y = input("Enter x and y coordinates (space separated): ").split(" ")
        target_x, target_y = int(x), int(y)

        start_time = time.time()

        while True:
            x, y = localiser.getPosition()

            angle = -(np.arctan2(y - target_y, x - target_x) * 180 / np.pi) - 90
            angle = (angle + 360) % 360  # Normalize angle to [0

            dis = np.sqrt((x - target_x) ** 2 + (y - target_y) ** 2)

            drive.move(angle, dis/2000, -imu.get_yaw()/720)

            if abs(x - target_x) < 10 and abs(y - target_y) < 10:
                print("Target reached!")
                drive.move(0, 0, 0)
                break

            if time.time() - start_time > 30:
                print("Timeout reached!")
                drive.move(0, 0, 0)
                break
            # localiser.printDebug()

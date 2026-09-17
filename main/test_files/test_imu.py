"""Test the IMU"""
from lib.imu import IMU
from time import sleep

def main(config):
    imu = IMU()
    imu.calibrate_yaw()

    while True:
        print(imu.get_yaw())
        sleep(0.033)
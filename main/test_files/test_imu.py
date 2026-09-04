"""Test the IMU"""
from lib.imu import IMU

def main(config):
    imu = IMU()
    imu.calibrate_yaw()

    while True:
        print(imu.get_yaw())
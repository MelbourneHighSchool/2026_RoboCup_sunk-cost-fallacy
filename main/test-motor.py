from lib.drive import Drive
from lib.config import Config
from lib.imu import IMU

config = Config()
drive = Drive.from_config(config)
imu = IMU()

yaw_zero = None
while yaw_zero is None:
    yaw_zero = imu.get_yaw()  # Wait until it's a normal value

while True:
    yaw = imu.get_yaw() - yaw_zero
    print(yaw)
    yaw_correct_amount = 0 if not yaw else yaw * 0.001
    drive.move(0, 0, yaw_correct_amount)
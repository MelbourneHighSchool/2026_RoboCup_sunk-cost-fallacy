from lib.Vision import Vision
from lib.imu import IMU
from lib.config import Config
from lib.interface import WSServer

import cv2
import numpy as np
import time

vision = Vision()
vision.load_config(Config())
vision.start()
imu = IMU(yawCallback=vision.yaw_callback)
imu.calibrate_yaw()

ws = WSServer()
ws.run()

mask = np.zeros(shape=(640, 640), dtype=np.uint8)
while True:
    data = np.reshape(vision.dbg_points, shape=[360, 2]).astype(int)
    for i in range(360):
        cv2.circle(mask, data[i], 5, 255, cv2.FILLED)
    ws.send_frame(mask)
    print(vision.estimated_pos_v[:])
    mask.fill(0)
    time.sleep(0.1)
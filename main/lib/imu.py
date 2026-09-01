import math
import threading
import time

import board
import busio

from adafruit_bno08x import BNO_REPORT_ACCELEROMETER, BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GAME_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

class IMU:
    def __init__(self, poll_interval=0.01, yawCallback = lambda yaw:None):
        self._poll_interval = poll_interval
        self._lock = threading.Lock()
        self._running = True
        self._latest_quaternion = None
        self._latest_yaw = None
        self._latest_acceleration = None
        self.yawCallback = yawCallback
        i2c = busio.I2C(board.SCL, board.SDA)
        self._bno = BNO08X_I2C(i2c)
        self._bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self._bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

        self.yaw_offset = 0
        self.calibrate_yaw()

    def wait_first(self, timeout):
        start_time = time.time()
        while not self._latest_yaw:
            if time.time() > start_time + timeout:
                return False
            continue
        return True

    def _update_loop(self):
        while self._running:
            try:
                quat_i, quat_j, quat_k, quat_real = self._bno.game_quaternion
                accel_x, accel_y, accel_z = self._bno.acceleration
                yaw = self._quaternion_to_yaw_degrees(quat_i, quat_j, quat_k, quat_real)
                self.yawCallback(yaw)
                with self._lock:
                    self._latest_quaternion = (quat_i, quat_j, quat_k, quat_real)
                    self._latest_yaw = yaw
                    self._latest_acceleration = (accel_x, accel_y, accel_z)
            except Exception as e:
                print(e)
                self._running = False
                # Keep the updater alive if a read occasionally fails.
                pass
            time.sleep(self._poll_interval)

    @staticmethod
    def _quaternion_to_yaw_degrees(x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    # we don't use quaternions
    # def get_latest_quaternion(self):
    #     with self._lock:
    #         return self._latest_quaternion

    def calibrate_yaw(self):
        self.wait_first(5)
        with self._lock:
            self.yaw_offset = self._latest_yaw

    def get_yaw(self):
        with self._lock:
            return self._latest_yaw - self.yaw_offset

    def get_acceleration(self):
        with self._lock:
            return self._latest_acceleration

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)


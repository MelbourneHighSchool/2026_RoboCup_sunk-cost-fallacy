import time
import math
import threading
from lib.motor import Motor
from lib.config import Config
from lib.imu import IMU
from math import radians, sin

SMOOTHING_TIME = 0.10
YAW_CORRECT_THRESHOLD = 3.0
YAW_CORRECT_SPEED = 0.2
POSSESSION_YAW_CORRECT_SPEED = 0.3
YAW_CORRECT_MAX_SPEED_THRESHOLD = 60 # If the error is greater than this angle yaw correction will be at the maximum speed.

# Dribbler Spin Constants
DRIBBLER_SPEED = -1.0
DRIBBLER_SPIN_YAW_CORRECT_THRESHOLD = 5.0
ORBIT_RADIUS_CM = 9.0  # distance from bot centre to ball
ORBIT_STRAFE_SPEED = 0.4  # strafe speed used while orbiting the ball in possession
RAD_TO_DEG = 180.0 / math.pi

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

def wrap_angle(theta):
    """Input an angle in degrees. Returns same angle but in [-180, 180)."""
    return (theta + 180) % 360 - 180

def capture_startup_yaw(imu:IMU, sample_count=25, sample_interval=0.02):
    """Average a short burst of IMU samples so startup yaw is not just the first reading."""
    print("Stabilizing IMU yaw reference...")
    sin_sum = 0.0
    cos_sum = 0.0
    samples = 0
    while samples < sample_count:
        yaw = imu.get_yaw()
        if yaw is not None:
            yaw_rad = math.radians(yaw)
            sin_sum += math.sin(yaw_rad)
            cos_sum += math.cos(yaw_rad)
            samples += 1
        time.sleep(sample_interval)
    return math.degrees(math.atan2(sin_sum, cos_sum))

class Drive:
    def __init__(self, imu:IMU=None, config:Config=None, motors=("ne", "se", "sw", "nw")):
        self.config = config if config is not None else Config()
        motors_config = self.config.get_value("motors", {})
        self.motors = {}
        self.current_direction = 0
        self.current_speed = 0
        self.target_direction = 0
        self.target_speed = 0
        self.target_rotation = 0
        self.target_yaw_correct_speed = YAW_CORRECT_SPEED
        self.possession = False
        self.orbiting = False
        self.orbit_yaw = 0.0
        self.last_orbit_time = time.monotonic()
        self.target_lock = threading.Lock()
        self.last_update_time = time.monotonic()
        self.imu = imu if imu is not None else IMU()
        self.initial_yaw = capture_startup_yaw(self.imu)
        self.yaw = 0
        for motor_direction in motors:
            motor_config = motors_config.get(motor_direction)
            try:
                self.motors[motor_direction] = Motor(motor_config["address"],
                                            elec_angle_offset=motor_config["elec_angle_offset"],
                                            sin_cos_centre=motor_config["sin_cos_centre"])
            except Exception as e:
                print(f"Error initializing motor at address {motor_config['address']}: {e}")

        self.drive_thread = threading.Thread(target=self._drive_loop, daemon=True)
        self.drive_thread.start()

    def move(self, angle, speed=0.5, rotation=0, possession=False): # rotation is the desired yaw value
        with self.target_lock:
            self.target_direction = angle
            self.target_speed = speed
            self.target_rotation = wrap_angle(rotation)
            self.possession = possession
            if possession:
                self.target_yaw_correct_speed = POSSESSION_YAW_CORRECT_SPEED
            else:
                self.target_yaw_correct_speed = YAW_CORRECT_SPEED

    @staticmethod
    def get_orbit_yaw_rate_deg(strafe_speed_ms, orbit_radius_cm):
        """Yaw rate (deg/s) to stay tidally locked while orbiting at strafe_speed_ms."""
        orbit_radius_m = orbit_radius_cm / 100.0
        if orbit_radius_m <= 0:
            return 0.0
        # Positive strafe right -> turn left (negative yaw).
        return -(strafe_speed_ms / orbit_radius_m) * RAD_TO_DEG
    
    def _drive_loop(self):
        motors_config = self.config.get_value("motors", {})
        while True:
            self._update_current_velocity()

            with self.target_lock:
                possession = self.possession
                target_rotation = self.target_rotation

            if (possession and abs(wrap_angle(target_rotation - self.yaw)) > DRIBBLER_SPIN_YAW_CORRECT_THRESHOLD):
                # Orbit the ball to reach target_rotation instead of rotating in place,
                # so the ball stays pinned against the dribbler.
                drive_direction, drive_speed, yaw_correction = self._orbit_step(target_rotation)
            else:
                self.orbiting = False
                drive_direction = self.current_direction
                drive_speed = self.current_speed
                yaw_correction = self._get_yaw_correction()

            for motor_direction, motor in self.motors.items():
                motor_config = motors_config.get(motor_direction)
                motor_angle = drive_direction - motor_config["angle_off"]
                motor_speed = drive_speed * -sin(radians(motor_angle))
                motor_speed = clamp(motor_speed - yaw_correction, -1.0, 1.0)
                motor.set_speed(motor_speed)

    def _orbit_step(self, target_rotation):
        """Single non-blocking orbit iteration.

        Strafes sideways while rotating in a coordinated way so the bot circles
        the held ball toward target_rotation. Returns (drive_direction,
        drive_speed, yaw_correction) for the drive loop to apply.
        """
        yaw_error = wrap_angle(target_rotation - self.yaw)

        now = time.monotonic()
        if not self.orbiting:
            self.orbiting = True
            self.orbit_yaw = self.yaw
            self.last_orbit_time = now
        dt = now - self.last_orbit_time
        self.last_orbit_time = now

        # Strafe toward the side that rotates the bot toward target_rotation.
        # Strafing right (+90) drives yaw negative (see get_orbit_yaw_rate_deg),
        # so a positive yaw error (needs yaw to increase) requires strafing left.
        orbit_sign = -1 if yaw_error >= 0 else 1
        strafe_dir = 90 * orbit_sign

        # Measure strafe speed along a fixed reference (+90, right) so the omega
        # sign is consistent for either orbit direction.
        strafe_speed_ms = self.get_speed_in_direction(90)
        omega_deg = self.get_orbit_yaw_rate_deg(strafe_speed_ms, ORBIT_RADIUS_CM)
        self.orbit_yaw = wrap_angle(self.orbit_yaw + omega_deg * dt)

        orbit_yaw_error = wrap_angle(self.orbit_yaw - self.yaw)
        yaw_correction = clamp(
            (orbit_yaw_error / YAW_CORRECT_MAX_SPEED_THRESHOLD) * YAW_CORRECT_SPEED,
            -YAW_CORRECT_SPEED,
            YAW_CORRECT_SPEED,
        )

        return strafe_dir, ORBIT_STRAFE_SPEED, yaw_correction

    def _get_yaw_correction(self):
        if not self._update_yaw():
            return 0

        with self.target_lock:
            target_rotation = self.target_rotation
            yaw_correct_speed = self.target_yaw_correct_speed

        yaw_error = wrap_angle(target_rotation - self.yaw)
        if abs(yaw_error) <= YAW_CORRECT_THRESHOLD:
            return 0

        return clamp(
            (yaw_error / YAW_CORRECT_MAX_SPEED_THRESHOLD) * yaw_correct_speed,
            -yaw_correct_speed,
            yaw_correct_speed,
        )

    def _update_yaw(self):
        raw_yaw = self.imu.get_yaw()
        if raw_yaw is None:
            return False

        self.yaw = wrap_angle(self.initial_yaw - raw_yaw)
        return True

    def _update_current_velocity(self):
        with self.target_lock:
            target_direction = self.target_direction
            target_speed = self.target_speed

        dx = math.cos(math.radians(self.current_direction)) * self.current_speed
        dy = math.sin(math.radians(self.current_direction)) * self.current_speed

        target_dx = math.cos(math.radians(target_direction)) * target_speed
        target_dy = math.sin(math.radians(target_direction)) * target_speed

        now = time.monotonic()
        dt = now - self.last_update_time
        self.last_update_time = now

        alpha = min(dt / SMOOTHING_TIME, 1.0)
        new_dx = dx + (target_dx - dx) * alpha
        new_dy = dy + (target_dy - dy) * alpha

        self.current_direction = math.degrees(math.atan2(new_dy, new_dx))
        self.current_speed = math.hypot(new_dx, new_dy)

    def get_body_velocity(self):
        """Return measured body-frame velocity (vx, vy) in m/s from wheel QDR.

        Inverts the omni mixing used in _drive_loop:
        w_i = vx*sin(angle_off_i) - vy*cos(angle_off_i)
        """
        motors_config = self.config.get_value("motors", {})
        ata_00 = 0.0
        ata_01 = 0.0
        ata_11 = 0.0
        atb_0 = 0.0
        atb_1 = 0.0

        for motor_direction, motor in self.motors.items():
            motor.update_quick_data_readout()
            wheel_speed = motor.get_wheel_speed()
            angle_off = math.radians(motors_config[motor_direction]["angle_off"])
            sin_a = math.sin(angle_off)
            cos_a = math.cos(angle_off)

            ata_00 += sin_a * sin_a
            ata_01 -= sin_a * cos_a
            ata_11 += cos_a * cos_a
            atb_0 += sin_a * wheel_speed
            atb_1 -= cos_a * wheel_speed

        det = ata_00 * ata_11 - ata_01 * ata_01
        if abs(det) < 1e-9:
            return 0.0, 0.0

        vx = (ata_11 * atb_0 - ata_01 * atb_1) / det
        vy = (ata_00 * atb_1 - ata_01 * atb_0) / det
        return vx, vy

    def get_speed_in_direction(self, direction_deg):
        """Return signed speed (m/s) along a body-frame direction."""
        vx, vy = self.get_body_velocity()
        direction = math.radians(direction_deg)
        return vx * math.cos(direction) + vy * math.sin(direction)

    def stop(self):
        with self.target_lock:
            self.target_direction = 0
            self.target_speed = 0
            self.target_rotation = self.yaw

        self.current_direction = 0
        self.current_speed = 0

        for motor in self.motors.values():
            motor.set_speed(0)
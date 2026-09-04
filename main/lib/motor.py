# Powerful brushless motor driver port
# Motor driver it originates from: https://oshwlab.com/andrewchen3128/bldc-driver-2-v4
# More motor driver documentation: https://github.com/Aw3someAndrew/SteelBar_CircuitPython_powerful_bldc_driver

"""
Brushless DC Motor Library ported to python
"""

import smbus2
import struct
import math

MAX_SPEED = 546133333
MAX_TORQUE = 100000

# Quick Data Readout (QDR) short format (FORMAT = 0x00) layout, 10 bytes total:
#   bytes 0-3 : uint32 POSITION (1 LSB = 1 electrical revolution)
#   bytes 4-7 : int32  SPEED    (1 LSB = 2^-16 POS/second, POS = electrical revs)
#   byte  8   : ERROR1
#   byte  9   : ERROR2
QDR_FORMAT_SHORT = 0x00
QDR_BYTE_COUNT = 10
SPEED_LSB_PER_ELEC_REV_PER_SEC = 1 << 16  # 2^16

WHEEL_DIAMETER_M = 0.05
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M

DEFAULT_POLE_PAIRS = 7
DEFAULT_GEAR_RATIO = 36.0

def clamp(value, a, b):
    return max(a, min(value, b))

class Motor:
    def __init__(self, address, bus_number: int = 1,
                 current_limit_FOC: int = 65536 * 2,
                 id_PID_constants: tuple[int] = (1500, 200),
                 iq_PID_constants: tuple[int] = (1500, 200),
                 speed_PID_constants: tuple[int] = (0.04, 0.0004, 0.03),
                 elec_angle_offset: int = 1510395136,
                 sin_cos_centre: int = 1251,
                 operating_mode_and_sensor: tuple[int] = (3, 1),
                 command_mode: int = 12,
                 max_speed: int = MAX_SPEED,
                 max_torque: int = MAX_TORQUE,
                 pole_pairs: int = DEFAULT_POLE_PAIRS,
                 gear_ratio: float = DEFAULT_GEAR_RATIO):
        self.i2c_address = address
        self.bus = smbus2.SMBus(bus_number)
        self.QDRformat = QDR_FORMAT_SHORT
        self.pole_pairs = pole_pairs
        self.gear_ratio = gear_ratio

        # QDR cache, refreshed by update_quick_data_readout()
        self.qdr_position = 0
        self.qdr_speed = 0
        self.qdr_error1 = 0
        self.qdr_error2 = 0

        # Default initialisation sequence
        self.set_current_limit_FOC(current_limit_FOC)
        self.set_id_PID_constants(*id_PID_constants)
        self.set_iq_PID_constants(*iq_PID_constants)
        self.set_speed_PID_constants(*speed_PID_constants)
        self.set_elec_angle_offset(elec_angle_offset)
        self.set_sin_cos_centre(sin_cos_centre)
        self.set_speed_limit(max_speed)
        self.max_torque = max_torque
        self.configure_operating_mode_and_sensor(*operating_mode_and_sensor)
        self.configure_command_mode(command_mode)
        self.set_quick_data_readout_format(self.QDRformat)

    def set_speed(self, speed: float):
        try:
            speed = int(self.max_speed * clamp(speed, -1.0, 1.0))
            data = struct.pack("<i", speed)
            self.bus.write_i2c_block_data(self.i2c_address, 0x12, list(data))
        except Exception as e:
            print(f"Error setting Speed: {e}")

    def set_iq_PID_constants(self, kp, ki):
        try:
            data = struct.pack("<ii", kp, ki)
            self.bus.write_i2c_block_data(self.i2c_address, 0x40, list(data))
        except Exception as e:
            print(f"Error setting Iq PID constants: {e}")

    def set_id_PID_constants(self, kp, ki):
        try:
            data = struct.pack("<ii", kp, ki)
            self.bus.write_i2c_block_data(self.i2c_address, 0x41, list(data))
        except Exception as e:
            print(f"Error setting Id PID constants: {e}")

    def set_speed_PID_constants(self, kp, ki, kd):
        try:
            data = struct.pack("<fff", kp, ki, kd)
            self.bus.write_i2c_block_data(self.i2c_address, 0x42, list(data))
        except Exception as e:
            print(f"Error setting Speed PID constants: {e}")

    def configure_operating_mode_and_sensor(self, operatingmode, sensortype):
        try:
            self.bus.write_byte_data(self.i2c_address, 0x20, operatingmode + (sensortype << 4))
        except Exception as e:
            print(f"Error configuring Operating Mode and Sensor: {e}")

    def configure_command_mode(self, commandmode):
        try:
            self.bus.write_byte_data(self.i2c_address, 0x21, commandmode)
        except Exception as e:
            print(f"Error configuring Command Mode: {e}")

    def set_speed_limit(self, speed_limit):
        try:
            self.max_speed = abs(speed_limit)
            data = struct.pack("<i", self.max_speed)
            self.bus.write_i2c_block_data(self.i2c_address, 0x34, list(data))
        except Exception as e:
            print(f"Error setting Speed Limit: {e}")

    def set_torque(self, torque):
        try:
            torque = int(self.max_torque * clamp(torque, -1.0, 1.0))
            data = struct.pack("<i", torque)
            self.bus.write_i2c_block_data(self.i2c_address, 0x11, list(data))
        except Exception as e:
            print(f"Error setting Torque: {e}")

    def set_position(self, position, elecangle):
        try:
            data = struct.pack("<I", position)
            self.bus.write_i2c_block_data(self.i2c_address, 0x13, list(data))
            self.send8bitvalue(elecangle)
        except Exception as e:
            print(f"Error setting Position: {e}")

    def set_current_limit_FOC(self, current):
        try:
            data = struct.pack("<i", current)
            self.bus.write_i2c_block_data(self.i2c_address, 0x33, list(data))
        except Exception as e:
            print(f"Error setting Current Limit FOC: {e}")

    def set_elec_angle_offset(self, ELECANGLEOFFSET):
        try:
            data = struct.pack("<I", ELECANGLEOFFSET)
            self.bus.write_i2c_block_data(self.i2c_address, 0x30, list(data))
        except Exception as e:
            print(f"Error setting ELECANGLEOFFSET: {e}")

    def set_sin_cos_centre(self, SINCOSCENTRE):
        try:
            data = struct.pack("<i", SINCOSCENTRE)
            self.bus.write_i2c_block_data(self.i2c_address, 0x32, list(data))
        except Exception as e:
            print(f"Error setting SINCOSCENTRE: {e}")
            
    def set_quick_data_readout_format(self, format_byte):
        try:
            self.QDRformat = format_byte
            self.bus.write_byte_data(self.i2c_address, 0x22, format_byte)
        except Exception as e:
            print(f"Error setting Quick Data Readout format: {e}")

    def update_quick_data_readout(self):
        """Refresh the cached Quick Data Readout (position, speed, errors).

        The driver returns the QDR on a plain I2C read with no preceding
        command, so this issues a register-less read of QDR_BYTE_COUNT bytes
        (matching the C++ updateQuickDataReadout()). Returns True on success.
        """
        try:
            msg = smbus2.i2c_msg.read(self.i2c_address, QDR_BYTE_COUNT)
            self.bus.i2c_rdwr(msg)
            data = bytes(msg)
            self.qdr_position = int.from_bytes(data[0:4], byteorder='little', signed=False)
            self.qdr_speed = int.from_bytes(data[4:8], byteorder='little', signed=True)
            self.qdr_error1 = data[8]
            self.qdr_error2 = data[9]
            return True
        except Exception as e:
            print(f"Error reading Quick Data Readout: {e}")
            return False

    def get_position_qdr(self):
        """Cached electrical-revolution position from the last QDR update."""
        return self.qdr_position

    def get_speed_qdr(self):
        """Cached raw speed from the last QDR update (1 LSB = 2^-16 POS/second)."""
        return self.qdr_speed

    def get_error1_qdr(self):
        return self.qdr_error1

    def get_error2_qdr(self):
        return self.qdr_error2

    def get_electrical_revs_per_sec(self):
        """Raw QDR speed converted to electrical revolutions/second."""
        return self.qdr_speed / SPEED_LSB_PER_ELEC_REV_PER_SEC

    def get_revs_per_sec(self):
        """Physical shaft revolutions/second.

        Electrical revs are divided by the pole-pair count (and gear ratio) to
        get physical output-shaft revolutions, per the driver documentation.
        """
        return self.get_electrical_revs_per_sec() / (self.pole_pairs * self.gear_ratio)

    def get_wheel_speed(self):
        """Wheel surface (strafe) speed in metres/second.

        surface speed = revs/second * wheel circumference.
        Call update_quick_data_readout() first to refresh the measurement.
        """
        return self.get_revs_per_sec() * WHEEL_CIRCUMFERENCE_M

    def read_wheel_speed(self):
        """Convenience: refresh the QDR and return wheel speed in m/s."""
        self.update_quick_data_readout()
        return self.get_wheel_speed()

    def read(self):
        """Backwards-compatible helper: refresh QDR and return raw values."""
        self.update_quick_data_readout()
        return [self.qdr_position, self.qdr_speed, self.qdr_error1, self.qdr_error2]
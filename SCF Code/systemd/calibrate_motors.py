
import time
import sys
import board
import busio
import select
from steelbar_powerful_bldc_driver import PowerfulBLDCDriver
from lib.config import Config

config = Config()

# Initialize variables
motor = [None] * 8
motor_address = [None] * 8
motormode = [0] * 8
motorcount = 0
setupmotorcount = 0

tempuint32 = 0
tempint32 = 0
tempfloat = 0.0

selectedmotor = 0
maxspeed = 0
postarget = 0.0
currentlimit = 0
pidconstant = 0.0
boundary = 0.0
pos = 0.0
clearfaults = False

def read_input():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def save_motor_calibration(address, elec_angle_offset, sin_cos_centre):
    motors_config = config.get_value("motors", {})

    for name, motor_config in motors_config.items():
        if motor_config.get("address") == address:
            motor_config["elec_angle_offset"] = elec_angle_offset
            motor_config["sin_cos_centre"] = sin_cos_centre
            config.set_value("motors", motors_config)
            config.save_config()
            print(f"Saved calibration for {name} motor to config.json")
            return

    print(f"Warning: no motor in config.json has address {address}; calibration was not saved.")

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

print("Please enter the number of motor drivers you want to control:")
tempuint32 = int(input())
if tempuint32 == 0 or tempuint32 > 8:
    print("Error motor count out of range, please reboot microcontroller to try again.")
    quit()
motorcount = tempuint32

setupmotorcount = 0
while setupmotorcount < motorcount:
    print(f"Please enter the i2c address of motor driver number {setupmotorcount}:")
    tempuint32 = int(input())
    if tempuint32 <= 7 or tempuint32 >= 120:
        print("Error invalid i2c address, please reboot microcontroller to try again.")
        quit()
    motor_address[setupmotorcount] = tempuint32
    motor[setupmotorcount] = PowerfulBLDCDriver(i2c, tempuint32)
    
    print(f"The firmware version of motor driver number {setupmotorcount} is: {motor[setupmotorcount].get_firmware_version()}")
    if motor[setupmotorcount].get_firmware_version() != 3:
        print("Error unsupported motor driver version, please check for updates, maybe check wiring and i2c configuration, reboot microcontroller to try again.")
        quit()

    setupmotorcount += 1

setupmotorcount = 0
while setupmotorcount < motorcount:
    motor[setupmotorcount].set_current_limit_foc(65536)  # set current limit to 1 amp (only works in FOC mode)
    motor[setupmotorcount].set_id_pid_constants(1500, 200)
    motor[setupmotorcount].set_iq_pid_constants(1500, 200)
    motor[setupmotorcount].set_speed_pid_constants(4e-2, 4e-4, 3e-2)  # Constants valid for FOC and Robomaster M2006 P36 motor only, see tuning constants document for more details
    motor[setupmotorcount].set_position_pid_constants(275, 0, 0)
    motor[setupmotorcount].set_position_region_boundary(250000)
    motor[setupmotorcount].set_speed_limit(10000000)
    
    motor[setupmotorcount].configure_operating_mode_and_sensor(15, 1)  # configure calibration mode and sin/cos encoder
    motor[setupmotorcount].configure_command_mode(15)  # configure calibration mode
    motor[setupmotorcount].set_calibration_options(300, 2097152, 50000, 500000)  # set calibration voltage to 300/3399*vcc volts, speed to 2097152/65536 elecangle/s, settling time to 50000/50000 seconds, calibration time to 500000/50000 seconds
    
    motor[setupmotorcount].start_calibration()  # start the calibration
    print(f"Starting calibration of motor {setupmotorcount}")
    while not motor[setupmotorcount].is_calibration_finished():  # wait for the calibration to finish, do not call any other motor driver functions while calibration is ongoing
        print(".", end="")
        sys.stdout.flush()
        time.sleep(0.5)
    print()  # print out the calibration results
    elec_angle_offset = motor[setupmotorcount].get_calibration_ELECANGLEOFFSET()
    sin_cos_centre = motor[setupmotorcount].get_calibration_SINCOSCENTRE()
    print(f"ELECANGLEOFFSET: {elec_angle_offset}")
    print(f"SINCOSCENTRE: {sin_cos_centre}")
    save_motor_calibration(motor_address[setupmotorcount], elec_angle_offset, sin_cos_centre)

    motor[setupmotorcount].configure_operating_mode_and_sensor(3, 1)  # configure FOC mode and sin/cos encoder
    motor[setupmotorcount].configure_command_mode(12)  # configure speed command mode
    motormode[setupmotorcount] = 12
    
    setupmotorcount += 1
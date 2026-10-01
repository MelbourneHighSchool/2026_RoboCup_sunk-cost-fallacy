from lib.config import Config
from lib import term

from enum import IntEnum

# Enums
# class 

# Select mode (only ask if no args are provided)
mode = term.radio_select(
    "Select mode",
    ["Components", "Programs", "Calibrate"],
    tip="1/2"
)
if mode is False:
    print("User quit")
    exit(0)

# Rest of setup
if mode == 0:
    preset = 0
    res = term.radio_select(
        "Select a component to test",
        ["Drive", "Vision", "Dribbler", "Interface", "IMU", "Switches", "Localisation", "Breakbeam"],
        descriptions=[
            "Test the drive system by giving motor commands.",
            "Test the vision system by detecting the ball and goals, and logging the information.",
            "Test the dribbler system by giving motor commands.",
            "Test the interface by logging messages from clients and sending messages to them.",
            "Test the IMU by printing its outputs and logging the time it took to start.",
            "Test the switches by printing their states.",
            "Test the localisation system by logging the robot's position.",
            "Test the breakbeam sensor with the dribbler."
        ],
        tip="2/2"
    )
elif mode == 1:
    res = term.multi_select(
        "Enable programs for testing",
        ["Ball Chasing", "Yaw Correction", "Goal Chasing", "Dribbler", "Kicker", "Stream"],
        descriptions=[
            "Enable the ball chasing program. If this is not selected, user commands will be used to control the robot.",
            "Enable the yaw correction program, to ensure the robot is always facing forward.",
            "Enable the goal chasing program, which will drive towards the goal when the ball is in possession.",
            "Enable the dribbler program, which will use the dribbler to maintain possession.",
            "Enable the kicker program, which will kick the ball when it is in possession and lined up with the goal.",
            "Enable the streaming interface, which will stream from the camera to the webinter."
        ],
        tip="2/2"
    )
elif mode == 2:
    res = term.radio_select(
        "Select a calibration mode",
        ["Camera", "Motors"],
        descriptions=[
            "Calibrate the camera by adjusting the HSV bounds for the ball and goals.",
            "Run the calibration program for the motors."
        ],
        tip="2/2"
    )

if res is False:
    print("User quit")
    exit(0)

# Load and run things
config = Config()

match mode:
    case 0:
        match res:
            case 1:
                from test_files import test_cam
                test_cam.main(config)
            case 3:
                from test_files import test_interface
                test_interface.main(config)
            case 4:
                from test_files import test_imu
                test_imu.main(config)
            case 5:
                from test_files import test_switches
                test_switches.main(config)
            case 6:
                from test_files import test_localise
                test_localise.main(config)
            case 7:
                from test_files import test_breakbeam
                test_breakbeam.main(config)
    case 1:
        match res:
            case 1:
                # from test_files import test_yawcor
                # test_yawcor.main(config)
                ...
    case 2:
        match res:
            case 0:
                from test_files import cal_cam
                cal_cam.main(config)
                
            case 1:
                # motor calibration
                ...
    case _:
        print("Error: Invalid mode")
        exit(1)

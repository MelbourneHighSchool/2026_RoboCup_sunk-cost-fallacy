from lib.config import Config
from lib import term

import argparse

# Setup parser
parser = argparse.ArgumentParser()
component_group = parser.add_argument_group("components", "test individual components").add_mutually_exclusive_group()
program_group = parser.add_argument_group("programs", "run multiple programs together for testing")

parser.add_argument("-c", "--calibrate", action="store_true", help="calibrate the camera")

component_group.add_argument("-d", "--drive", action="store_true", help="test drive")
component_group.add_argument("-v", "--vision", action="store_true", help="test vision")
component_group.add_argument("-r", "--dribbler", action="store_true", help="test dribbler")
component_group.add_argument("-i", "--interface", action="store_true", help="test interface")

program_group.add_argument("-b", "--ball-chasing", action="store_true", help="enable ball chasing")
program_group.add_argument("-y", "--yaw-correction", action="store_true", help="enable yaw correction")
program_group.add_argument("-g", "--goal-chasing", action="store_true", help="enable goal chasing")
program_group.add_argument("-k", "--kicker", action="store_true", help="enable kicker")
program_group.add_argument("-s", "--stream", action="store_true", help="enable streaming interface")

args = parser.parse_args()

# Select mode (only ask if no args are provided)
if not any([v for _, v in args._get_kwargs()]):
    mode = term.radio_select(
        "Select mode",
        ["Components", "Programs", "Calibrate"],
        tip="1/2"
    )
    if mode is False:
        print("User quit")
        exit(0)
else:
    components = any([args.drive, args.vision, args.dribbler, args.interface])
    programs = any([args.ball_chasing, args.yaw_correction, args.goal_chasing, args.kicker, args.stream])
    calibrate = args.calibrate
    if components + programs + calibrate > 1:
        print("Error: Can only select one of components, programs, or calibrate")
        exit(1)
    else:
        if components:
            mode = 0
        elif programs:
            mode = 1
        else:
            mode = 2

# Rest of setup
if mode == 0:
    preset = 0
    if any([args.drive, args.vision, args.dribbler, args.interface]):
        preset = [args.drive, args.vision, args.dribbler, args.interface].index(True)
    res = term.radio_select(
        "Select a component to test",
        ["Drive", "Vision", "Dribbler", "Interface"],
        descriptions=[
            "Test the drive system by giving motor commands.",
            "Test the vision system by detecting the ball and goals, and logging the information.",
            "Test the dribbler system by giving motor commands.",
            "Test the interface by logging messages from clients and sending messages to them."
        ],
        preset=preset,
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
        preset=[args.ball_chasing, args.yaw_correction, args.goal_chasing, args.dribbler, args.kicker, args.stream],
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
                from _test_files import test_cam
                test_cam.main(config)
    case 1:
        ...
    case 2:
        match res:
            case 0:
                from _test_files import cal_cam
                cal_cam.main(config)
                
            case 1:
                # motor calibration
                ...
    case _:
        print("Error: Invalid mode")
        exit(1)
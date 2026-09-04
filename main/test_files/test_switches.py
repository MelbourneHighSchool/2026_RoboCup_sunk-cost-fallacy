from lib.switch import Switch
from time import sleep

def main(config):
    switches = config.get_value("switches")

    goal_conf = switches.get("goal")
    goal_switch = Switch(goal_conf["pin"], goal_conf["on_high"])

    run_conf = switches.get("run")
    run_switch = Switch(run_conf["pin"], run_conf["on_high"])

    while True:
        goal_state = goal_switch.read()
        run_state = run_switch.read()

        print(f"Goal switch: {goal_state}, Run switch: {run_state}")
        sleep(0.05)
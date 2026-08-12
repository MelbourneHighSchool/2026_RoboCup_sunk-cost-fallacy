from lib.switch import Switch
import board
import time

pause_switch = Switch(board.D16)
goal_switch = Switch(board.D12)

while True:
    pause_state = "RUNNING" if pause_switch.read() else "PAUSED"
    goal_state = "YELLOW goal" if goal_switch.read() else "BLUE goal"
    print(f"pause: {pause_state} | goal: {goal_state}")
    time.sleep(0.2)
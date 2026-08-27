"""
Contains localizer class
"""
import subprocess, threading
from math import radians, degrees
from lib.imu import IMU
from lib.tof import ToF
from time import sleep
class Localizer:
    """
    Localization Module

    Takes tof and imu sensors during init, creates thread to read sensors and update hillclimb thread
    
    Use getPositionAndBearing to read output
    """
    tofDistanceFromCenter = 50
    def __init__(self, tofs:list[ToF], imu:IMU):
        self._alive = True
        self.tofs = tofs
        self.imu = imu
        self.cppModule = subprocess.Popen(
            ["lib/localizeFast"], #decide filetype later
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        self.cppIOLock = threading.Lock()
        self.readerThread = threading.Thread(target=self._tofUpdator, daemon=True)
        self.readerThread.start()
    def kill(self):
        "Murder is bad, idk why you'd want to do this"
        self._alive = False
        with self.cppIOLock:
            self.cppModule.stdin.write("e\n")
            self.cppModule.stdin.flush()

    def _tofUpdator(self):
        # integrate imu and tof update loops with this update loop for less latency?
        readings = [2000] * 8
        while self._alive:
            instruction = "i "
            for i, tof in enumerate(self.tofs):
                reading = tof.read()
                if reading:
                    readings[i] = reading + 40
            #TODO
            instruction += " ".join(map(str, readings)) + " "
            instruction += str(radians(self.imu.get_yaw())) + "\n"
            with self.cppIOLock:
                self.cppModule.stdin.write(instruction)
                self.cppModule.stdin.flush()
                # print("===========================\n\nYo guys we wrote B)\n\n======================")
            #sync with update cycle maybe idk
            sleep(0.01)

    def getPositionAndBearing(self):
        with self.cppIOLock:
            self.cppModule.stdin.write("o\n")
            self.cppModule.stdin.flush()
        output = (self.cppModule.stdout.readline(), self.cppModule.stdout.readline()), self.cppModule.stdout.readline(), self.cppModule.stdout.readline()
        # print(output)
        x, y = output[0]
        x = float(x[:-1])
        y = float(y[:-1])
        bearing = degrees(float(output[1][:-1]))
        sensor_data = output[2]
        return ((x, y), bearing, sensor_data)

if __name__ == "__main__":
    loc = Localizer([1,2,3,4,5,6,7,8], "imu") # TODO TODO TODO

    try:
        while True:
            print(f"Yo we got a successful reading up here {loc.getPositionAndBearing()}")
    finally:
        loc.kill()


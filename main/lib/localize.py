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
        for i, tof in enumerate(self.tofs):
            tof.callback = self.updateFunctionGenerator(i)
        self.imu = imu
        self.imu.yawCallback = self.updateFunctionGenerator(8) 
        self.cppModule = subprocess.Popen(
            ["lib/localizeFast"], #decide filetype later
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        self.cppIOLock = threading.Lock()
    def updateFunctionGenerator(self, i):
        "Generates update function to assign to callback"
        return lambda dist: self.updateReading(i, dist)
    def updateReading(self, index, value):
        "Updates reading from sensor i with value value"
        with self.cppIOLock:
            self.cppModule.stdin.write(f"i {index} {value}\n")
            self.cppModule.stdin.flush()

    def kill(self):
        "Murder is bad, idk why you'd want to do this"
        with self.cppIOLock:
            self.cppModule.stdin.write("e\n")
            self.cppModule.stdin.flush()

    def getPosition(self):
        with self.cppIOLock:
            self.cppModule.stdin.write("o\n")
            self.cppModule.stdin.flush()
        output = (self.cppModule.stdout.readline(), self.cppModule.stdout.readline()),\
            self.cppModule.stdout.readline(), self.cppModule.stdout.readline(),\
            self.cppModule.stdout.readline()
        # print(output)
        x, y = output[0]
        x = float(x[:-1])
        y = float(y[:-1])
        # bearing = degrees(float(output[1][:-1]))
        # sensor_data = output[2]
        return (x, y)

if __name__ == "__main__":
    loc = Localizer([1,2,3,4,5,6,7,8], "imu") # TODO TODO TODO

    try:
        while True:
            print(f"Yo we got a successful reading up here {loc.getPositionAndBearing()}")
    finally:
        loc.kill()


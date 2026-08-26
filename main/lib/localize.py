"""
Contains localizer class
"""
import subprocess, threading
from math import radians
from imu import IMU
from time import sleep
class Localizer:
    """
    Localization Module

    Takes tof and imu sensors during init, creates thread to read sensors and update hillclimb thread
    
    Use getPositionAndBearing to read output
    """
    tofDistanceFromCenter = 50
    def __init__(self, tofs:list, imu:IMU):
        self._alive = True
        self.tofs = tofs
        self.imu = imu
        self.cppModule = subprocess.Popen(
            ["localizeFast.bin"], #decide filetype later
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
        while self._alive:
            instruction = "i "
            for tof in self.tofs: 
                reading = tof #TODO read value from tof
                instruction += str(reading + self.tofDistanceFromCenter) + " "
            #TODO
            instruction += str(radians(self.imu.get_yaw())) + "\n"
            with self.cppIOLock:
                self.cppModule.stdin.write(instruction)
                self.cppModule.stdin.flush()      
            #sync with update cycle maybe idk
            sleep(0.01)

    def getPositionAndBearing(self):
        with self.cppIOLock:
            self.cppModule.stdin.write("o\n")
            self.cppModule.stdin.flush()
        return (self.cppModule.stdout.readline(), self.cppModule.stdout.readline()), self.cppModule.stdout.readline()

if __name__ == "__main__":
    loc = Localizer([1,2,3,4,5,6,7,8], "imu") # TODO TODO TODO

    try:
        while True:
            print(loc.getPositionAndBearing())
    finally:
        loc.kill()


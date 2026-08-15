import ctypes as ct 
from random import random
import subprocess
from time import perf_counter
#tofData = (ct.c_float * 8)()

process = subprocess.Popen(
    ["localizeFast"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    text=True,
    bufsize=1
)
while True:
    action = input()
    t = perf_counter()
    process.stdin.write(action.strip() + "\n")
    process.stdin.flush()
    if action[0] == "o":
        o1, o2, o3 = process.stdout.readline(), process.stdout.readline(), process.stdout.readline()
    elif action[0] == "e":
        break
    print(t - perf_counter())
    if action[0] == "o":
        print(o1, o2, o3)

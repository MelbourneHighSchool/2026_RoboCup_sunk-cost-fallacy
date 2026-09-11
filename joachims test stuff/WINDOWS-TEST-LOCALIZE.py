from random import random
import subprocess, sys, os
from time import perf_counter
import pygame as pg
from math import pi, radians, sin, cos
from colorsys import hsv_to_rgb
#tofData = (ct.c_float * 8)()
progPath = sys.path[0].replace("\\","/")
mainPath = progPath.removesuffix("joachims test stuff") + "/main/"
print(mainPath)
sensorData = []
with open(progPath + "/tof data.txt", "r") as file:
    for line in file.readlines():
        _, _, imuString, tofString, _ = line.strip().split(",")
        print(tofString, imuString)
        a = []
        for v in tofString.strip().removeprefix("'").removesuffix("\\n'").strip().split(" "):
            a.append(int(v))
        sensorData.append((a,0))#float(imuString.strip().removeprefix("'").removesuffix("\\n'"))))
pg.init()
clock = pg.time.Clock()
dis = pg.display.set_mode((460, 608))

def cvtPoint(p:pg.Vector2):
    return (p + pg.Vector2(1820,2430)) / 8
process = subprocess.Popen(
    [mainPath + "lib/localizeFast.exe"],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    text=True,
    bufsize=1
)
dataIdx = 0
process.stdin.write("i 8 0\n")
process.stdin.flush()
try:
    while True:
        for e in pg.event.get():
            if e.type == pg.QUIT:
                raise SystemExit
        dis.fill((50,150,100))
        print(sensorData[dataIdx][1])
        process.stdin.write(f"i {i} {d}\n")
        process.stdin.flush()
        pos = pg.Vector2((process.stdout.readline(), process.stdout.readline()))
        for i, d in enumerate(sensorData[dataIdx][0]):
            print(i)
            process.stdin.write(f"i {i} {d}\n")
            process.stdin.flush()
            a = (pi * i / 4) - sensorData[dataIdx][1]
            pg.draw.aaline(dis, 
                            hsv_to_rgb(i/8,1,1),
                            cvtPoint(pg.Vector2(0,0)),
                            cvtPoint(pg.Vector2(sin(a),cos(a)) * d)
                          )
                            
        pg.display.flip()
        
        clock.tick(100)
        dataIdx = (dataIdx + 1)
        if dataIdx >= len(sensorData):
            dataIdx -= len(sensorData)
            p = True
            while p:
                for e in pg.event.get():
                    if e.type == pg.QUIT:
                        raise SystemExit
                    elif e.type == pg.KEYDOWN:
                        p = False
finally:
    process.stdin.write("e")
    process.stdin.flush()
    pg.quit()
while True:
    action = input()
    t = perf_counter()
    process.stdin.write(action.strip() + "\n")
    process.stdin.flush()
    if action[0] == "o":
        print()
        for i in range(5):
            print(process.stdout.readline().strip())
        print()
    elif action[0] == "e":
        break
    print(t - perf_counter())


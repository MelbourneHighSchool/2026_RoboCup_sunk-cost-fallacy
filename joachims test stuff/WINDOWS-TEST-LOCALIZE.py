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
fRect = pg.rect.Rect(cvtPoint(pg.Vector2(-910, -1215)), pg.Vector2(1820/8, 2430/8))
g1Rect = pg.rect.Rect(cvtPoint(pg.Vector2(-225, -1215)), pg.Vector2(450/8, 226/8))
g2Rect = pg.rect.Rect(cvtPoint(pg.Vector2(-225, 988)), pg.Vector2(450/8, 226/8))

debug = [[],[],0]
try:
    while True:
        for e in pg.event.get():
            if e.type == pg.QUIT:
                raise SystemExit
        print(sensorData[dataIdx][1])
        process.stdin.write(f"o\nd\n")
        process.stdin.flush()
        pos = pg.Vector2((int(process.stdout.readline()), int(process.stdout.readline())))
        for i in range(3):
            debug[i] = process.stdout.readline().split(" ")
        dis.fill((20,20,20))
        pg.draw.rect(dis,(33,100,67),fRect)
        pg.draw.rect(dis,(255,255,0),g1Rect)
        pg.draw.rect(dis,(0,255,255),g2Rect)
        for p in [ (225,  1215,   940),   # top goal, right post
                    (-225, 1215,   940),   # top goal, left post
                    (225, -1215,  -940),   # bottom goal, right post
                    (-225,-1215,  -940),   # bottom goal, left post
                ]:
            pg.draw.line(dis,(20,20,20),cvtPoint(pg.Vector2(p[0],p[1])),cvtPoint(pg.Vector2(p[0],p[2])))
        for i, d in enumerate(sensorData[dataIdx][0]):
            
            process.stdin.write(f"i {i} {d}\n")
            process.stdin.flush()
            a = (pi * i / 4) - sensorData[dataIdx][1]
            pg.draw.aaline(dis, 
                            (255,255,255) if debug[1][i] == "0" else (255,0,0),  #hsv_to_rgb(360 * i / 8,255,255),
                            cvtPoint(pos),
                            cvtPoint(pos + pg.Vector2(sin(a),cos(a)) * d)
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


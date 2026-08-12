from lib.drive import Drive
from time import sleep

d = Drive()

d.move(0, 0.1)
sleep(2)
d.move(90, 0.1)
sleep(2)
d.move(180, 0.1)
sleep(2)
d.move(270, 0.1)
sleep(2)
d.stop()
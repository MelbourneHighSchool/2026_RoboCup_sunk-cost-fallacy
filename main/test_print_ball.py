from lib.Vision import Vision
import time
import cv2
import numpy as np

vision = Vision()
vision.start()
start_time = time.time()
while time.time() - start_time < 60.0:
    try:
        cv2.imwrite("/var/www/html/frame.jpg", cv2.cvtColor(vision.camera.latest_frame, cv2.COLOR_BGR2RGB))
        time.sleep(0.1)
    except KeyboardInterrupt:
        break

print("Done")
vision.camera.stop()
vision.broadcaster.deinit()
print("Deinit done")
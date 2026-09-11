from lib.Vision import Vision
import numpy as np
import time

vision = Vision()

# Get an initial image and put it into shared memory
vision.camera.pre_callback = None
vision.camera.start()
frame: np.ndarray = vision.camera.capture_array()
frame_size = vision.camera.frame_size
frame_buffer = vision.camera.frame_buffer

for i in range(3):
    slc = slice(frame_size * i, frame_size * (i+1))
    frame_buffer.buf[slc] = frame.flatten()

# Start vision without camera
vision.start()
vision.camera.stop()

# Continuously set latest frame event
c_new_frame = vision.camera.c_new_frame
v_latest_timestamp = vision.camera.v_latest_timestamp
for i in range(2000):
    v_latest_timestamp.value += 0.001
    c_new_frame.notify_all()
    time.sleep(0.001)
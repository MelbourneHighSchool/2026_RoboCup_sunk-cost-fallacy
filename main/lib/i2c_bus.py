"""Process-wide I2C serialization.

Motors, ToF sensors and the IMU all share the single physical I2C bus (bus 1)
but each opens its own handle (smbus2 for motors/ToF, busio/Blinka for the IMU).
Without a common lock their transactions collide on the wire, saturating the bus
and delaying motor writes / staling IMU reads. Every I2C transaction in this
project should be wrapped with this lock so the bus is accessed serially.
"""

import threading

I2C_LOCK = threading.RLock()

# Implementation in logic.py
# self.tofs = ToFs(0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x5f)

"""Background reader for the SteelBar time-of-flight sensor."""

import threading
import time

try:
    from smbus2 import SMBus, i2c_msg
except ImportError as exc:
    raise ImportError(
        "tof.py requires smbus2. Install it with `pip install smbus2`."
    ) from exc

from lib.i2c_bus import I2C_LOCK


class ToFs:
    def __init__(self, addresses:list, bus_number=1, poll_interval=0.002, callback=lambda idx, distance:None):
        self._addresses = addresses
        self._poll_interval = poll_interval
        self._lock = I2C_LOCK
        self._running = True
        self._last_sequences = [None] * len(addresses)
        self._latest_distance = None
        self._bus = SMBus(bus_number)
        self.callback = callback
        self._thread = threading.Thread(target=self._update_loop, daemon=True)
        self._thread.start()

    def _read_sensor(self, address):
        with self._lock:
            write = i2c_msg.write(address, [0x10])
            read = i2c_msg.read(address, 5)
            self._bus.i2c_rdwr(write, read)
        data = list(read)

        if len(data) != 5:
            raise RuntimeError(f"Expected 5 bytes from ToF sensor, got {len(data)}")

        sequence = data[0]
        distance = int.from_bytes(bytes(data[1:5]), byteorder="little", signed=True)
        return sequence, distance

    def _update_loop(self):
        addr_i = 0
        while self._running:
            try:
                address = self._addresses[addr_i]
                sequence, distance = self._read_next_measurement(address)
                if distance is None:
                    continue
                self.callback(addr_i, distance)
                self._latest_distance = distance
            finally:
                addr_i = (addr_i + 1) % len(self._addresses)
                time.sleep(self._poll_interval)

    def _read_next_measurement(self, addr_i):
        sequence, distance = self._read_sensor(self._addresses[addr_i])
        changed = sequence != self._last_sequences[addr_i]
        self._last_sequences[addr_i] = sequence
        if changed:
            return sequence, distance
        return None, None

    def read(self):
        with self._lock:
            return self._latest_distance

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        with self._lock:
            self._bus.close()

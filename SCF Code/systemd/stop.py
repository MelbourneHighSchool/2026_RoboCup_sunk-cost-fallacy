import struct

import smbus2

from lib.config import Config


SPEED_REGISTER = 0x12
BUS_NUMBER = 1


def main():
    config = Config()
    zero_speed = list(struct.pack("<i", 0))
    motors = config.get_value("motors", {})

    with smbus2.SMBus(BUS_NUMBER) as bus:
        for name, motor_config in motors.items():
            address = motor_config.get("address")
            if address is None:
                continue
            try:
                bus.write_i2c_block_data(address, SPEED_REGISTER, zero_speed)
            except Exception as error:
                print(f"Error stopping {name} motor at address {address}: {error}")


if __name__ == "__main__":
    main()
import time

from lib.imu import IMU

PRINT_INTERVAL_SECONDS = 0.05


def format_value(value):
    if value is None:
        return "----"
    return f"{value:7.2f}"


def format_accel(acceleration):
    if acceleration is None:
        return "ax=---- ay=---- az=----"
    ax, ay, az = acceleration
    return f"ax={format_value(ax)} ay={format_value(ay)} az={format_value(az)}"


if __name__ == "__main__":
    imu = IMU()

    try:
        print("Reading IMU yaw and acceleration. Ctrl+C to stop.")
        while True:
            yaw = imu.get_yaw()
            acceleration = imu.get_acceleration()
            print(
                f"\ryaw={format_value(yaw)} deg  {format_accel(acceleration)} m/s^2",
                end="",
                flush=True,
            )
            time.sleep(PRINT_INTERVAL_SECONDS)
    except KeyboardInterrupt:
        print()
    finally:
        imu.close()
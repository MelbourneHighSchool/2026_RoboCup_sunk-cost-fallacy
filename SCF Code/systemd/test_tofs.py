import time

from lib.tof import ToFs


PRINT_INTERVAL_SECONDS = 0.05


def format_distance(distance):
    if distance is None:
        return "----"

    return f"{distance:4d}"


if __name__ == "__main__":
    tofs = ToFs()

    try:
        print("Reading 8 ToF sensors. Ctrl+C to stop.")

        while True:
            distances = tofs.read()

            readings = "  ".join(
                f"tof{i + 1}: {format_distance(distance)}"
                for i, distance in enumerate(distances)
            )

            print(
                f"\r{readings}",
                end="",
                flush=True
            )

            time.sleep(PRINT_INTERVAL_SECONDS)

    except KeyboardInterrupt:
        print()

    finally:
        tofs.close()
import digitalio

class BreakBeam:
    break_beam = None
    def __init__(self, pin):
        self.break_beam = digitalio.DigitalInOut(pin)
        self.break_beam.direction = digitalio.Direction.INPUT
        self.break_beam.pull = digitalio.Pull.UP

    def read(self):
        return not self.break_beam.value
import board
from digitalio import DigitalInOut, Direction, Pull

class Switch:
    def __init__(self, pin:str, on_high:bool=True):
        self.on_high = on_high

        self.switch_pin = DigitalInOut(board.__getattribute__(pin))
        self.switch_pin.direction = Direction.INPUT
        self.switch_pin.pull = Pull.UP

    def read(self):
        if self.on_high:
            return self.switch_pin.value
        else:
            return not self.switch_pin.value
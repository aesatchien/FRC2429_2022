#from wpilib.buttons import Button
from wpilib.buttons import Button
from wpilib import Joystick

class AxisButton(Button):
    """
    A custom button that is used when pretending an axis button is digital.
    Note - don't forget the init for the parent!  Will crash in 2020 w/o it.
    """

    def __init__(self, joystick, axis):
        #Button.__init__()
        super().__init__()
        self.joystick = joystick
        self.axis = axis
        self.threshold = 0.03

    def get(self):
        return self.joystick.getRawAxis(self.axis) > self.threshold


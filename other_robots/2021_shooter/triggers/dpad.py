#from wpilib.buttons import Button
# from wpilib.command import Button
from wpilib.buttons import Button
from wpilib import Joystick

class Dpad(Button):
    """
    Custom button that knows when the dpad is pressed in any direction.
    Can use it to continuously drive with the dpad
    """

    def __init__(self, joystick):
        # Button.__init() does not work...
        #Button.__init__()
        super().__init__()

        self.joystick = joystick

    def get(self):
        return self.joystick.getPOV(0) != -1

    def angle(self):
        return self.joystick.getPOV(0)
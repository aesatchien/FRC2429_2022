#from wpilib.buttons import Button
from wpilib.command import Button
from wpilib import Joystick

class POVButton(Button):
    """
    A custom button that is triggered when the dpad is used.
    povButtonUp =   POVButton(stick, 0)
    povButtonDown =   POVButton(stick, 180)
    povButtonRight =   POVButton(stick, 90)
    povButtonLeft =   POVButton(stick, 270)
    I it returns -1 if it is not being used.
    Note - don't forget the init for the parent!  Will crash in 2020 w/o it.
    """

    def __init__(self, joystick, angle):
        # Button.__init() does not work...
        #Button.__init__()
        super().__init__()

        self.joystick = joystick
        self.angle = angle

    def get(self):
        return self.joystick.getPOV(0) == self.angle

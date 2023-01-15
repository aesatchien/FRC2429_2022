from wpilib import AddressableLED



class Led():

    def __init__(self, robotcontainer):
        self.robotcontainer = robotcontainer
        self.led = AddressableLED(4)
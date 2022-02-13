import commands2
from subsystems.pixy import Pixy
from subsystems.drivetrain import Drivetrain

class DriveToBall(commands2.CommandBase):
    def __init__(self, pixy: Pixy, drive: Drivetrain) -> None:
        super().__init__()
        self.pixy = pixy

    # def initialize(self) -> None:

    # def execute(self) -> None:

    # def end(self) -> None:

    # def isFinished(self) -> bool:

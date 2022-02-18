import commands2
from subsystems.pixy import Pixy
from subsystems.drivetrain import Drivetrain
from wpilib import SmartDashboard

# temporary flag for debugging this command
DEBUG_COMMAND = False

# command for searching for ball when none are in view
class FindBall(commands2.CommandBase):
    def __init__(self, search_speed: float, pixy: Pixy, drive: Drivetrain) -> None:
        super().__init__()

        # command uses the pixy for vision input and drivetrain for rotation
        self.pixy = pixy
        self.drive = drive
        self.addRequirements([self.pixy, self.drive])

        #
        self.search_speed = search_speed

        if DEBUG_COMMAND:
            SmartDashboard.putNumber('FindBall/search_speed', search_speed)

    def initialize(self) -> None:
        self.drive.arcadeDrive(0, 0)
        self.drive.resetGyro()

    def execute(self) -> None:
        if DEBUG_COMMAND:
            self.search_speed = SmartDashboard.getNumber('FindBall/search_speed', 0)

        # check if a ball has been detected
        (ball_detected, _, _) = self.pixy.getBallValues()

        # if not, then just rotate until one is spotted
        if not ball_detected:
            self.drive.arcadeDrive(0, self.search_speed)

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        (ball_detected, _, _) = self.pixy.getBallValues()

        # end command once ball has been detected
        return ball_detected
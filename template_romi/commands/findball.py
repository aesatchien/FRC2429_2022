import commands2
from subsystems.pixy import Pixy
from subsystems.drivetrain import Drivetrain
from wpilib import SmartDashboard

class FindBall(commands2.CommandBase):
    def __init__(self, speed: float, pixy: Pixy, drive: Drivetrain) -> None:
        super().__init__()

        self.speed = speed
        self.pixy = pixy
        self.drive = drive

        self.addRequirements([self.pixy, self.drive])

    def initialize(self) -> None:
        self.drive.arcadeDrive(0, 0)
        self.drive.resetGyro()

    def execute(self) -> None:
        (ball_detected, _, _) = self.pixy.getBallValues()

        if not ball_detected:
            self.drive.arcadeDrive(0, self.speed)

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        (ball_detected, _, _) = self.pixy.getBallValues()
        SmartDashboard.putBoolean('ball_debug/found_ball', ball_detected)

        return ball_detected
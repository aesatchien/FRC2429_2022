"""
CJH re-writing the romi TurnDegrees template  2022 0121
Because it:
1) was not actually using the gyro
2) was not verbose (did not tell us what it was trying to do)
3) was not ready to be turned into a PID
"""

import math
import commands2
from wpilib import Timer
from subsystems.drivetrain import Drivetrain

class TurnDegreesFFWD(commands2.CommandBase):
    def __init__(self, speed: float, degrees: float, drive: Drivetrain) -> None:
        """Creates a new TurnDegrees. This command will turn your robot for a desired rotation (in
        degrees) and rotational speed.  It is NOT a PID.  It is just a feedforward until setpoint is reached.

        :param speed:   The speed which the robot will drive. Only absolute value matters.
        :param degrees: Degrees to turn.  Positive is CW, negative is CCW.
        :param drive:   The drive subsystem on which this command will run
        """
        super().__init__()

        self.target = degrees
        self.speed = speed
        self.drive = drive
        self.addRequirements(self.drive)  # we won't share the drivetrain

        # initialize some stuff we should track
        self.angle = 0
        self.start_angle = 0  # this way we don't have to re-initialize the gyro each time
        self.start_time = 0

        # sanity check - make speed match direction we are turning
        if self.target > 0:
            self.speed = math.fabs(self.speed)
        elif self.target < 0:
            self.speed = -math.fabs(self.speed)
        else:
            pass

    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        # Set motors to stop, read gyro value for starting point
        self.drive.arcadeDrive(0, 0)
        self.start_angle = self.drive.gyro.getAngleZ()

        # let's have a decent message telling us what we're doing
        self.start_time = Timer.getFPGATimestamp()
        print("\n" + f"** Started {self.__class__.__name__} turning {self.target}deg at {self.start_time:.1f} s **", flush=True)

    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""
        self.drive.arcadeDrive(0, self.speed)
        self.angle = self.drive.gyro.getAngleZ() - self.start_angle  # measure how far along we have gone

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        end_time = Timer.getFPGATimestamp()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.__class__.__name__} - turned {self.angle:.1f} after {end_time - self.start_time:.1f} s **", flush=True)

        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # with absolute value you don't need an if statement on the direction
        tolerance = 1  # degree
        # if error < tolerance we can finish, i.e setpoint - actual is less than tolerance then we can stop
        if math.fabs(self.target) - math.fabs(self.angle) < tolerance:
            return True

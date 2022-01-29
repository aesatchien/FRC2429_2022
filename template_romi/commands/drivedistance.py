# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import commands2
from wpilib import Timer
from subsystems.drivetrain import Drivetrain


class DriveDistance(commands2.CommandBase):
    def __init__(self, speed: float, inches: float, drive: Drivetrain) -> None:
        """Creates a new DriveDistance. This command will drive your your robot for a desired distance at
        a desired speed.

        :param speed:  The speed at which the robot will drive
        :param inches: The number of inches the robot will drive
        :param drive:  The drivetrain subsystem on which this command will run
        """
        super().__init__()

        self.distance = inches
        self.speed = speed
        self.drive = drive
        self.addRequirements(drive)

    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        self.drive.arcadeDrive(0, 0)
        self.drive.resetEncoders()

        # let's have a decent message telling us what we're doing
        self.start_time = Timer.getFPGATimestamp()
        print("\n" + f"** Started {self.__class__.__name__} driving {self.distance} at {self.start_time:.1f} s **", flush=True)

    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""
        self.drive.arcadeDrive(self.speed, 0)

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        end_time = Timer.getFPGATimestamp()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.__class__.__name__} - drove {self.drive.getAverageDistanceInch():.1f} after {end_time - self.start_time:.1f} s **", flush=True)

        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        """Returns true when the command should end."""
        # Compare distance travelled from start to desired distance
        return abs(self.drive.getAverageDistanceInch()) >= self.distance

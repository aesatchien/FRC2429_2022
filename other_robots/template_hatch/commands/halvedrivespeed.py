import time
from wpilib import SmartDashboard
import commands2

from subsystems.drivesubsystem import DriveSubsystem


class HalveDriveSpeed(commands2.CommandBase):
    def __init__(self, container, drive: DriveSubsystem) -> None:
        super().__init__()
        self.setName('halve_drive_speed')
        self.drive = drive
        self.robot_container = container

    def initialize(self) -> None:
        self.drive.setMaxOutput(0.5)
        self.start_time = round(time.time() - self.robot_container.get_start_time(), 1)
        print("\n" + f"** Started {self.__class__.__name__} / {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time:2.2f} s **")

    def end(self, interrupted: bool) -> None:
        self.drive.setMaxOutput(1.0)
        end_time = round(time.time() - self.robot_container.get_start_time(), 1)
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time} s after {round(end_time - self.start_time, 1)} s **")
        SmartDashboard.putString("alert", f"** {message} {self.getName()} at {end_time} s after {round(end_time - self.start_time, 1)} s **")

import math

import commands2
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain
import constants
import rev

class AutoRotateSparkmax(commands2.CommandBase):
    """
    Rotate the robot based on a calibrated FWD/REV motion of the drivetrain, not on the IMU
    SmartMotion could be much cleaner than using the IMU
    """
    def __init__(self, container, drive: Drivetrain, target='ball', degrees=0) -> None:
        super().__init__()
        self.setName('AutoRotateSparkmax')
        self.container = container
        self.drive = drive
        self.addRequirements(drive)  # commandsv2 version of requirements

        self.target = target
        self.distance_per_degree = 2.8 / 360  # calibration is 2.8?m for a 360 degree spin.  So 36 deg is 0.28m.
        self.velocity = 0.5  # m/s
        self.drive_time = 0  # how long we spin
        self.degrees = degrees

    def initialize(self) -> None:
        self.drive.drive.feed()

        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.drive_time = 0  # exit right away if no targets
        if self.target == 'ball':
            (ball_detected, rotation_offset, distance) = self.container.robot_vision.getBallValues()
            if ball_detected:
                self.drive_time = abs(self.distance_per_degree * rotation_offset / self.velocity)
                self.drive.smart_velocity(velocity=self.velocity * math.copysign(1, rotation_offset), spin=True)

        elif self.target == 'hub':
            (hub_detected, rotation_offset, distance) = self.container.robot_vision.getHubValues()
            if hub_detected:
                self.drive_time = abs(self.distance_per_degree * rotation_offset / self.velocity)
                self.drive.smart_velocity(velocity=self.velocity * math.copysign(1, rotation_offset), spin=True)

        elif self.target == 'degrees':
            self.drive_time = abs(self.distance_per_degree * self.degrees / self.velocity)
            self.drive.smart_velocity(velocity=self.velocity * math.copysign(1, self.degrees), spin=True)


    def execute(self) -> None:
        self.drive.drive.feed()

    def isFinished(self) -> bool:
        #error = self.degrees * self.distance_per_degree - abs(self.drive.left_encoder.getPosition() - self.encoder_start_position)
        #return abs(error) < 0.1
        return self.container.get_enabled_time() - self.start_time > self.drive_time


    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
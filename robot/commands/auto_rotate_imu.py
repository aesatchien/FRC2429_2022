import commands2
from math import copysign
from wpilib import SmartDashboard
from wpimath.controller import PIDController

SmartDashboard.putNumber('/Imu/kf', 0)
SmartDashboard.putNumber('/Imu/kp', 0)
SmartDashboard.putNumber('/Imu/kd', 0)

class AutoRotateImu(commands2.CommandBase):
    """
    Rotate the robot based a PID control with the heading coming from the drivetrain's IMU
    """

    def __init__(self, container, drive, source='degrees', degrees=45) -> None:
        super().__init__()
        self.setName('AutoRotateIMU')
        self.container = container
        self.drive = drive
        self.degrees = degrees
        self.source = source
        self.controller = PIDController(0.0025, 0, 0.0001)
        self.feed_forward = 0.2
        self.tolerance = 1.5

        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.drive.arcade_drive(0, 0)
        self.start_angle = self.drive.navx.getAngle()

        # allow ball or hub to override our choice of turning
        if self.source == 'ball':
            (ball_detected, rotation_offset, distance) = self.container.robot_vision.getBallValues()
            self.degrees = rotation_offset if ball_detected else 0
        elif self.source == 'hub':
            (hub_detected, rotation_offset, distance) = self.container.robot_vision.getHubValues()
            self.degrees = rotation_offset if hub_detected else 0

        if abs(self.degrees) > 30:
            self.feed_forward = 0.22
            self.controller = PIDController(0.0025, 0, 0.0001)
        else:
            self.feed_forward = 0.20
            self.controller = PIDController(0.0025, 0, 0.0001)

        self.controller.reset()
        self.controller.setSetpoint(self.degrees)

        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s with source {self.source} and heading {self.start_angle:3.1f} with target {self.degrees} **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        relative_orientation = self.drive.navx.getAngle() - self.start_angle
        output = self.controller.calculate(relative_orientation)
        output += copysign(1, output) * self.feed_forward

        self.drive.arcade_drive(0, output)

    def isFinished(self) -> bool:
        relative_orientation = self.drive.navx.getAngle() - self.start_angle
        return abs(relative_orientation - self.degrees) < self.tolerance

    def end(self, interrupted: bool) -> None:
        self.drive.arcade_drive(0, 0)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        relative_orientation = self.drive.navx.getAngle() - self.start_angle
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s at {relative_orientation} **", flush=True)
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



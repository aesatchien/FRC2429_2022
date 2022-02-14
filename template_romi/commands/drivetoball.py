import commands2
from wpimath import controller
from subsystems.pixy import Pixy
from subsystems.drivetrain import Drivetrain
from wpilib import SmartDashboard

class DriveToBall(commands2.CommandBase):
    def __init__(self, pixy: Pixy, drive: Drivetrain) -> None:
        super().__init__()

        self.pixy = pixy
        self.drive = drive
        self.controller = controller.PIDController(Kp=0.0, Ki=0.0, Kd=0.0)
        self.addRequirements([self.pixy, self.drive])

    def initialize(self) -> None:
        self.drive.arcadeDrive(0, 0)

        self.drive.resetEncoders()
        self.drive.resetGyro()

    def execute(self) -> None:
        (ball_detected, angle_offset, target_distance) = self.pixy.getBallValues()

        self.controller.setP(SmartDashboard.getNumber('ball_debug/kp', 0))
        self.controller.setI(SmartDashboard.getNumber('ball_debug/ki', 0))
        self.controller.setD(SmartDashboard.getNumber('ball_debug/kd', 0))

        if ball_detected:
            current_angle = self.drive.gyro.getAngleZ()
            pid_output = self.controller.calculate(current_angle, current_angle + angle_offset)
            self.drive.arcadeDrive(0, pid_output)

            SmartDashboard.putNumber('ball_debug/pid_output', pid_output)
            SmartDashboard.putNumber('ball_debug/angle_z', current_angle)

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        return False
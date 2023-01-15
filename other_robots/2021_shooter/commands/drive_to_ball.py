from wpilib.command import Command
from wpilib import Timer, SmartDashboard

class DriveToBall(Command):
    def __init__(self, robot):
        Command.__init__(self, name='DriveToBall')

        self.robot = robot
        self.vision = robot.vision
        self.drivetrain = robot.drivetrain

        self.requires(self.drivetrain)

    def initialize(self) -> None:
        self.drivetrain.arcade_drive(0, 0)

        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp(), 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.robot.enabled_time:2.2f} s **")

    def execute(self) -> None:
        (ball_detected, rotation_offset, distance) = self.vision.getBallValues()
        SmartDashboard.putBoolean('/DriveToBall/ball_detected', ball_detected)

        if ball_detected or abs(rotation_offset) > 5:
            SmartDashboard.putNumber('/DriveToBall/rotation_offset', rotation_offset)
            SmartDashboard.putNumber('/DriveToBall/distance', distance)

            speed_ratio = abs(rotation_offset) / 30
            added_speed = 0.1
            min_speed = 0.2
            speed = min_speed + added_speed * speed_ratio

            angle_is_negative = rotation_offset < 0
            output = -speed if angle_is_negative else speed

            self.drivetrain.arcade_drive(0, output)

    def isFinished(self) -> bool:
        return False

    def end(self, message='Ended') -> None:
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print(f"** {message} {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

        SmartDashboard.putString('/Vision/TestKey', 'ended')

    def interrupted(self) -> None:
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')
from math import copysign
import commands2
from wpilib import SmartDashboard

class AutoFetchBall(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive, vision) -> None:
        super().__init__()
        self.setName('AutoFetchBall')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.vision = vision

        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.drive.arcade_drive(0, 0)

        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        self.drive.feed()

        (ball_detected, rotation_offset, distance) = self.vision.getBallValues()
        SmartDashboard.putBoolean('/AutoFetchBall/ball_detected', ball_detected)

        error = abs(rotation_offset)
        
        if ball_detected and error > 5:
            SmartDashboard.putNumber('/AutoFetchBall/rotation_offset', rotation_offset)
            SmartDashboard.putNumber('/AutoFetchBall/distance', distance)

            kp = 0.003
            min_speed = 0.2
            twist_output = min_speed + kp * error
            twist_output *= copysign(1, rotation_offset)

            thrust_output = 0.2

            self.drivetrain.arcade_drive(thrust_output, twist_output)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
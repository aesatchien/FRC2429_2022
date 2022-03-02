import commands2
from wpilib import SmartDashboard


class AutoRotateImu(commands2.CommandBase):
    """
    Rotate the robot based a PID control with the heading coming from the drivetrain's IMU
    """

    def __init__(self, container, drive, degrees) -> None:
        super().__init__()
        self.setName('Auto Rotate IMU')
        self.container = container
        self.drive = drive
        self.addRequirements(drive)  # commandsv2 version of requirements
        self.degrees = degrees

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



import commands2
from wpilib import SmartDashboard
from subsystems.drivetrain import Drivetrain

class AutoRotateSparkmax(commands2.CommandBase):
    """
    Rotate the robot based on a calibrated FWD/REV motion of the drivetrain, not on the IMU
    SmartMotion could be much cleaner than using the IMU
    """
    def __init__(self, container, drive: Drivetrain, degrees) -> None:
        super().__init__()
        self.setName('Auto Rotate Sparkmax')
        self.container = container
        self.drive = drive
        self.addRequirements(drive)  # commandsv2 version of requirements

        self.degrees = degrees

        self.distance_per_degree = 2.8 / 360  # calibration is 2.8?m for a 360 degree spin


    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.pid_controllers = [controller.getPIDController() for controller in self.drive.controllers]


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



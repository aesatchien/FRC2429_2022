import commands2
from wpilib import SmartDashboard
import wpimath.geometry as geo
import constants


class AutoSetPose(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, pose=None) -> None:
        super().__init__()
        self.setName('AutoSetPose')  # change this to something appropriate for this command
        self.container = container
        # self.addRequirements(self.container.)  # commandsv2 version of requirements
        self.pose = pose

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Fired {self.getName()} at {self.start_time} s **", flush=True)

        # either keep a table here or pull from constants
        self.x = constants.k_start_x
        self.y = constants.k_start_y
        self.heading = constants.k_start_heading
        self.container.robot_drive.reset_odometry(geo.Pose2d(self.x, self.y, geo.Rotation2d().fromDegrees(self.heading)))

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass



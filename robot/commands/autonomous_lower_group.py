import commands2
from wpilib import SmartDashboard
from commands.autonomous_ramsete import AutonomousRamsete
import trajectory_io

class AutonomousLowerGroup(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('auto lower group')  # change this to something appropriate for this command
        self.container = container

        # next step - reverse to a ball
        self.traj_1 = trajectory_io.generate_quick_trajectory(x=1.7, y=0, heading=0, velocity=3, reverse=True)
        self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory', trajectory=self.traj_1))

        # next step - return to hub

        self.traj_2 = trajectory_io.generate_quick_trajectory(x=2.3, y=0, heading=0, velocity=3, reverse=False)
        self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory', trajectory=self.traj_2))


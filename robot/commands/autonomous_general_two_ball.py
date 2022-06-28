import commands2
from commands.auto_ramsete import AutoRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.indexer_hold import IndexerHold
from commands.drive_wait import DriveWait
from commands.auto_rotate_imu import AutoRotateImu

import trajectory_io

class AutonomousGeneralTwoBall(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutonomousGeneralTwoBall')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 6.0
        self.intake_speed = 0.6

        self.path_velocity = 1


        # open and start intake
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed, force='on'))

        # next step - reverse to a ball
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=2600, force='on'))

        # drive straight backwards to intake ball
        status, self.traj_1 = trajectory_io.generate_quick_trajectory(x=1.2, y=0, heading=0, velocity=self.path_velocity, reverse=False)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=True, source='trajectory', trajectory=self.traj_1))

        # hopefully pick up a ball
        self.addCommands(DriveWait(container=self.container, duration=0.1))

        # next step - return to hub with the shooter on
        self.addCommands(DriveWait(container=self.container, duration=0.2))

        # next step - shoot twice
        self.addCommands(AutoRotateImu(container=self.container, drive=self.container.robot_drive, source='hub'))
        self.addCommands(IndexerHold(self.container, self.container.robot_indexer, voltage=3, shot_time=5, autonomous=True))

        # close up, turn off shooter - maybe
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0, force='off'))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0, force='off'))
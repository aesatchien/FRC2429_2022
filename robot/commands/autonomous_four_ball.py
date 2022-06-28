import commands2
from commands.autonomous_two_ball import AutonomousTwoBall
from commands.auto_ramsete import AutoRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.indexer_toggle import IndexerToggle
from commands.drive_wait import DriveWait
from commands.auto_track_hub import AutoTrackHub
from commands.indexer_hold import IndexerHold

import constants
import trajectory_io

class AutonomousFourBall(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutonomousFourBall')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 6.0
        self.intake_speed = 0.6

        trajectory_files = ['two_ball_traversal', 'terminal_to_shot', 'ball_backup', 'terminal_to_shot2']
        path_velocity = constants.k_path_velocity

        # run the initial two-ball command
        self.addCommands(AutonomousTwoBall(self.container))

        # reverse and turn towards terminal to set up next trajectory

        trajectory = trajectory_io.generate_trajectory(path_name=trajectory_files[2], velocity=path_velocity, display=True, save=False)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=False,
                                     dash=False, source='trajectory', trajectory=trajectory, course=trajectory_files[0]))

        # make sure intake system is running
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed, force='on'))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=3, force='on'))

        # drive to terminal, getting open ball on the way
        trajectory = trajectory_io.generate_trajectory(path_name=trajectory_files[0], velocity=path_velocity, display=True, save=False)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=False,
                                     dash=False, source='trajectory', trajectory=trajectory, course=trajectory_files[0]))
        self.addCommands(DriveWait(container=self.container, duration=0.1))


        # we now have two balls, drive back to take the shot
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=3, force='off'))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=constants.k_shooter_speed, force='on'))

        trajectory = trajectory_io.generate_trajectory(path_name=trajectory_files[3], velocity=path_velocity, display=True, save=False)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=False,
                                     dash=False, source='trajectory', trajectory=trajectory, course=trajectory_files[3]))

        # rotate towards the hub
        self.addCommands(DriveWait(container=self.container, duration=0.3))
        self.addCommands(AutoTrackHub(container=self.container, drive=self.container.robot_drive, shooter=self.container.robot_shooter, vision=self.container.robot_vision, pneumatics=self.container.robot_pneumatics, autonomous=True))

        self.addCommands(IndexerHold(self.container, self.container.robot_indexer, voltage=3, shot_time=1.5, autonomous=True))


        # close up, turn off shooter
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0, force='off'))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=2.5, force='on'))

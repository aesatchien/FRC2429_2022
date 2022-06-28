import commands2
from commands.auto_ramsete import AutoRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.indexer_hold import IndexerHold
from commands.auto_set_pose import AutoSetPose

import constants
import trajectory_io

class AutonomousTwoBall(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutonomousTwoBall')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 6.0
        self.intake_speed = 0.7
        # self.index_pulse_on = 0.2
        # self.index_pulse_off = 0.5
        self.path_velocity = constants.k_path_velocity

        trajectory_src = 'hub_to_ball'

        # try to reset the pose to the current position
        self.addCommands(AutoSetPose(self.container))

        # open and start intake
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed, force='on'))

        # next step - reverse to a ball
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=2600, force='on'))

        # follow trajectory from starting position against hub to intake lower ball
        trajectory = trajectory_io.generate_trajectory(path_name=trajectory_src, velocity=self.path_velocity, display=True, save=False)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=False,
                                     dash=False, source='trajectory', trajectory=trajectory, course=trajectory_src))

        # next step - shoot twice
        self.addCommands(IndexerHold(self.container, self.container.robot_indexer, voltage=3.5, shot_time=3, autonomous=True))

        # close up, turn off shooter - maybe
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0, force='off'))
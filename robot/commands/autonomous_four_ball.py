import commands2
from wpilib import SmartDashboard
from commands.autonomous_two_ball import AutonomousTwoBall
from commands.auto_ramsete import AutoRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.indexer_toggle import IndexerToggle
from commands.drive_wait import DriveWait
from commands.auto_rotate_imu import AutoRotateImu
from commands.auto_pickup import AutoPickup
from commands.auto_shoot import AutoShoot
from commands.indexer_hold import IndexerHold

import constants
import trajectory_io


class AutonomousFourBall(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutonomousFourBall')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 3.0
        self.intake_speed = 0.6
        # self.index_pulse_on = 0.15
        # self.index_pulse_off = 0.4
        trajectory_files = ['two_ball_traversal', 'terminal_to_shot']
        path_velocity = constants.k_path_velocity

        # run the initial two-ball command

        self.addCommands(AutonomousTwoBall(self.container))

        # spin so we can make a faster trajectory
        self.addCommands(AutoRotateImu(container=self.container, drive=self.container.robot_drive,
                                       source='degrees', degrees=110))

        # make sure intake system is running
        # self.addCommands(IndexerHold(container=self.container, indexer=self.container.robot_indexer, voltage=3))
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed, force='on'))

        # drive to terminal, getting open ball on the way
        trajectory = trajectory_io.generate_trajectory(path_name=trajectory_files[0], velocity=path_velocity, display=True, save=False)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=False,
                                     dash=False, source='trajectory', trajectory=trajectory, course=trajectory_files[0]))
        self.addCommands(DriveWait(container=self.container, duration=0.1))


        # we now have two balls, drive back to take the shot
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=constants.k_shooter_speed, force='on'))

        trajectory = trajectory_io.generate_trajectory(path_name=trajectory_files[1], velocity=path_velocity, display=True, save=False)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=False,
                                     dash=False, source='trajectory', trajectory=trajectory, course=trajectory_files[1]))


        # rotate towards the hub
        self.addCommands(DriveWait(container=self.container, duration=0.2))
        self.addCommands(AutoRotateImu(self.container, self.container.robot_drive, source='hub'))

        self.addCommands(IndexerHold(self.container, self.container.robot_indexer, voltage=3, shot_time=1.75, autonomous=True))

        # start the shooter and move to the hub
        self.addCommands(DriveWait(container=self.container, duration=0.1))

        # get close enough to see it
        #status, self.traj_1 = trajectory_io.generate_quick_trajectory(x=1.3, y=0, heading=0, velocity=2, reverse=True)
        #self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=True, source='trajectory', trajectory=self.traj_1))

        # call the shooting routine
        # self.addCommands(AutoShoot(self.container))

        # close up, turn off shooter
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0, force='off'))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=0, force='off'))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0, force='off'))
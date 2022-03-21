import commands2
from wpilib import SmartDashboard
from commands.autonomous_two_ball import AutonomousTwoBall
from commands.auto_ramsete import AutoRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.indexer_toggle import IndexerToggle
from commands2 import WaitCommand
from commands.auto_rotate_imu import AutoRotateImu
from commands.auto_pickup import AutoPickup
from commands.auto_shoot import AutoShoot

import trajectory_io


class AutonomousStageTwo(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutonomousStageTwo')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 3.0
        self.intake_speed = 0.75
        self.index_pulse_on = 0.15
        self.index_pulse_off = 0.4

        # run the initial two-ball command
        self.addCommands(AutonomousTwoBall(self.container))

        # turn to look at other ball
        self.addCommands(AutoRotateImu(self.container, self.container.robot_drive, source='degrees', degrees=100))

        # drive to ball
        status, self.traj_1 = trajectory_io.generate_quick_trajectory(x=1, y=0, heading=0, velocity=2, reverse=False)
        self.addCommands(
            AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=True, source='trajectory',
                        trajectory=self.traj_1))

        # pick up ball
        self.addCommands(AutoPickup(self.container))

        # rotate towards the hub

        self.addCommands(AutoRotateImu(self.container, self.container.robot_drive, source='degrees', degrees=-75))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0, force='off'))

        # start the shooter and move to the hub
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=1900, force='on'))

        # get close enough to see it
        status, self.traj_1 = trajectory_io.generate_quick_trajectory(x=1.3, y=0, heading=0, velocity=2, reverse=True)
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, relative=True, source='trajectory',
                                     trajectory=self.traj_1))

        # call the shooting routine
        self.addCommands(AutoShoot(self.container))

        # close up, turn off shooter
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0, force='off'))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=0, force='off'))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0, force='off'))
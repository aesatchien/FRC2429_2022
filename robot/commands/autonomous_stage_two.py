import commands2
from wpilib import SmartDashboard
from commands.auto_ramsete import AutonomousRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.indexer_toggle import ToggleFeed
from commands2 import WaitCommand
from commands.auto_rotate_imu import AutoRotateImu
from commands.auto_pickup import AutonomousPickup
from commands.auto_shoot import AutonomousShooting

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

        # open and start intake
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed, force='on'))

        # next step - reverse to a ball
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=2050, force='on'))
        status, self.traj_1 = trajectory_io.generate_quick_trajectory(x=1.3, y=0, heading=0, velocity=1.8, reverse=False)
        self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory',trajectory=self.traj_1))

        # hopefully pick up a ball
        # self.addCommands(ToggleShooter(self.container, self.container.robot_shooter, rpm=2000))
        self.addCommands(WaitCommand(0.3))

        # next step - return to hub with the shooter on
        # status, self.traj_2 = trajectory_io.generate_quick_trajectory(x=1, y=0, heading=0, velocity=3, reverse=True)
        # self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory', trajectory=self.traj_2))
        self.addCommands(WaitCommand(.3))

        # next step - shoot twice - this could just call the autonomous shooting routine for brevity
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0).
                         andThen(WaitCommand(self.index_pulse_off)))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0).
                         andThen(WaitCommand(self.index_pulse_off)))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))

        self.addCommands(WaitCommand(.2))

        # turn to look at other ball
        self.addCommands(AutoRotateImu(self.container, self.container.robot_drive, source='degrees', degrees=100))

        # drive to ball
        status, self.traj_1 = trajectory_io.generate_quick_trajectory(x=1, y=0, heading=0, velocity=2, reverse=False)
        self.addCommands(
            AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory',
                              trajectory=self.traj_1))

        # pick up ball
        self.addCommands(AutonomousPickup(self.container))

        # rotate towards the hub

        self.addCommands(AutoRotateImu(self.container, self.container.robot_drive, source='degrees', degrees=-75))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0, force='off'))

        # start the shooter and move to the hub
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=1900, force='on'))

        # get close enough to see it
        status, self.traj_1 = trajectory_io.generate_quick_trajectory(x=1.3, y=0, heading=0, velocity=2, reverse=True)
        self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory',
                              trajectory=self.traj_1))

        # call the shooting routine
        self.addCommands(AutonomousShooting(self.container))

        # close up, turn off shooter
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0, force='off'))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0, force='off'))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0, force='off'))
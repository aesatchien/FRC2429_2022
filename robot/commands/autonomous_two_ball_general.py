import commands2
from wpilib import SmartDashboard
from commands.autonomous_ramsete import AutonomousRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.toggle_feed import ToggleFeed
from commands.autonomous_pickup import AutonomousPickup
from commands.autonomous_shooting import AutonomousShooting
from commands2 import WaitCommand

import trajectory_io

class AutonomousTwoBallGeneral(commands2.SequentialCommandGroup):

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutonomousTwoBallGeneral')  # change this to something appropriate for this command
        self.container = container

        self.addCommands(AutonomousPickup(self))
        # self.addCommands(WaitCommand(1)) wait?
        self.addCommands(AutonomousShooting(self))

        # self.indexer_speed = 3.0
        # self.intake_speed = 0.65
        # self.index_pulse_on = 0.15
        # self.index_pulse_off = 0.4

        # open and start intake
        # self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        # self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed))

        # next step - reverse to a ball


        # self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=2000))
        # self.traj_1 = trajectory_io.generate_quick_trajectory(x=2.3, y=0, heading=0, velocity=3, reverse=False)
        # self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory', trajectory=self.traj_1))

        # hopefully pick up a ball
        #self.addCommands(ToggleShooter(self.container, self.container.robot_shooter, rpm=2000))
        # self.addCommands(WaitCommand(0.1))

        # next step - return to hub with the shooter on
        # self.traj_2 = trajectory_io.generate_quick_trajectory(x=1.7, y=0, heading=0, velocity=3, reverse=True)
        # self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='trajectory', trajectory=self.traj_2))
        # self.addCommands(WaitCommand(1))

        # next step - shoot twice
        # self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
        #                  andThen(WaitCommand(self.index_pulse_on)))
        # self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0).
        #                  andThen(WaitCommand(self.index_pulse_off)))
        # self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
        #                  andThen(WaitCommand(self.index_pulse_on)))

        # close up, turn off shooter
        # self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics))
        # self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0))
        # self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0))
        # self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0))
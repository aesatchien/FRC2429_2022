import commands2
from wpilib import SmartDashboard
from commands.autonomous_ramsete import AutonomousRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.toggle_feed import ToggleFeed
from commands.auto_rotate_sparkmax import AutoRotateSparkmax
from commands2 import WaitCommand

import trajectory_io

class AutonomousPickup(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('auto pickup')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 3.0
        self.intake_speed = 0.65
        self.index_pulse_on = 0.15
        self.index_pulse_off = 0.4

        # open and start intake
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed))

        # next step - spin to ball
        self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='ball'))
        self.addCommands(WaitCommand(0.2))
        #self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='ball'))
        #self.addCommands(WaitCommand(0.2))
        
        # ToDo: add a set distance for driving - basically get the distance from the vision system and make a trajectory
        self.addCommands(AutonomousRamsete(container=self.container, drive=self.container.robot_drive, source='ball'))

        # next step - pulse in the ball
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0).
                         andThen(WaitCommand(self.index_pulse_off)))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))

        # shut down intake
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0))
        self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0))
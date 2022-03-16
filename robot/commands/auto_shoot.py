import commands2
from wpilib import SmartDashboard
from commands.auto_ramsete import AutoRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.shooter_toggle import ShooterToggle
from commands.indexer_toggle import IndexerToggle
from commands.auto_rotate_sparkmax import AutoRotateSparkmax
from commands2 import WaitCommand

import trajectory_io

class AutoShoot(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutoShoot')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 3.0
        self.intake_speed = 0.65
        self.index_pulse_on = 0.15
        self.index_pulse_off = 0.3

        # close and stop intake, fire up shooter
        # self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0, force='off'))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=1950, force='on'))

        # next step - spin to hub
        self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='hub'))
        self.addCommands(WaitCommand(0.2))
        #self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='hub'))
        #self.addCommands(WaitCommand(0.2))

        # ToDo: add a set distance for shooting - basically get the distance from the vision system and make a trajectory
        # adjust distance
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, source='hub'))

        # align again - just in case
        self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='hub'))

        # next step - shoot twice
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=0).
                         andThen(WaitCommand(self.index_pulse_off)))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=0).
                         andThen(WaitCommand(self.index_pulse_off)))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=0).
                         andThen(WaitCommand(self.index_pulse_off)))

        # close up, turn off shooter
        #self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0, force='off'))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=0, force='off'))
        self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0, force='off'))
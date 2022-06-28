import commands2
from commands.auto_ramsete import AutoRamsete
from commands.intake_position_toggle import IntakePositionToggle
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.indexer_toggle import IndexerToggle
from commands.auto_rotate_imu import AutoRotateImu
from commands2 import WaitCommand

import trajectory_io

class AutoPickup(commands2.SequentialCommandGroup):  # change the name for your command

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('AutoPickup')  # change this to something appropriate for this command
        self.container = container
        self.indexer_speed = 3.0
        self.intake_speed = 0.65
        self.index_pulse_on = 0.15
        self.index_pulse_off = 0.4

        # open and start intake
        self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='extend'))
        self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=self.intake_speed, force='on'))

        # next step - spin to ball
        #self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='ball'))
        self.addCommands(AutoRotateImu(self.container, self.container.robot_drive, source='ball'))
        self.addCommands(WaitCommand(0.5))
        #self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='ball'))
        #self.addCommands(WaitCommand(0.2))
        
        # ToDo: add a set distance for driving - basically get the distance from the vision system and make a trajectory
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, source='ball'))

        # next step - pulse in the ball
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=0).
                         andThen(WaitCommand(self.index_pulse_off)))
        self.addCommands(IndexerToggle(self.container, self.container.robot_indexer, voltage=self.indexer_speed).
                         andThen(WaitCommand(self.index_pulse_on)))

        # shut down intake
        #self.addCommands(IntakePositionToggle(self.container, self.container.robot_pneumatics, force='retract'))
        #self.addCommands(IntakeMotorToggle(self.container, self.container.robot_intake, velocity=0.0))
        #self.addCommands(ToggleFeed(self.container, self.container.robot_indexer, voltage=0))
        #self.addCommands(ShooterToggle(self.container, self.container.robot_shooter, rpm=0))
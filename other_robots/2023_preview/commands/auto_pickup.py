import commands2
from commands.auto_ramsete import AutoRamsete
from commands.auto_rotate_sparkmax import AutoRotateSparkmax
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

        # next step - spin to ball
        self.addCommands(AutoRotateImu(self.container, self.container.robot_drive, source='ball'))
        self.addCommands(WaitCommand(0.5))

        # ToDo: add a set distance for driving - basically get the distance from the vision system and make a trajectory
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, source='ball'))

        # next step - pulse in the ball


        # shut down intake

import commands2
from commands.auto_ramsete import AutoRamsete
from commands.auto_rotate_sparkmax import AutoRotateSparkmax
from commands2 import WaitCommand

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

        # next step - spin to hub
        self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='hub'))
        self.addCommands(WaitCommand(0.2))


        # ToDo: add a set distance for shooting - basically get the distance from the vision system and make a trajectory
        # adjust distance
        self.addCommands(AutoRamsete(container=self.container, drive=self.container.robot_drive, source='hub'))

        # align again - just in case
        self.addCommands(AutoRotateSparkmax(self.container, self.container.robot_drive, target='hub'))

        # next step - shoot twice


        # close up, turn off shooter

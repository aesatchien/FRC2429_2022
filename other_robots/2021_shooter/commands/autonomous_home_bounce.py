# a group of commands strung together autonomously
from wpilib.command import CommandGroup
from commands.autonomous_ramsete_simple import AutonomousRamseteSimple

class AutonomousBounce(CommandGroup):
    """ allows for stringing together autonomous commands """

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='auto_bounce')
        # bounce
        self.addSequential(AutonomousRamseteSimple(robot, path='bounce_pw1', relative=True, reset_telemetry=True))  # initial move forwards
        self.addSequential(AutonomousRamseteSimple(robot, path='bounce_pw2', relative=False, reset_telemetry=False))  # second move reverse
        self.addSequential(AutonomousRamseteSimple(robot, path='bounce_pw3', relative=False, reset_telemetry=False))  # third more forwards
        self.addSequential(AutonomousRamseteSimple(robot, path='bounce_pw4', relative=False, reset_telemetry=False))  # fourth move backwards


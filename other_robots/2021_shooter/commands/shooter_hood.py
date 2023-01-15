from wpilib.command import Command
from wpilib import Timer, SmartDashboard
import math


class ShooterHood(Command):
    """
    The raises and lowers the hood.
    """

    def __init__(self, robot, power=0.1, button=None):
        Command.__init__(self, name='shooter_hood')
        # self.requires(robot.shooter)
        self.robot = robot
        self.power = power
        self.button = button

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp()-self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.robot.enabled_time:2.2f} s **")

    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above to tweak speeds and correction
        """
        self.robot.shooter.change_elevation(self.power)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        self.robot.shooter.set_hood_motor(0)
        print(f"** {message} {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')

from wpilib.command import Command
from wpilib import Timer, SmartDashboard

class ShooterFire(Command):
    """
    This function shoots a ball -
    """

    def __init__(self, robot, button, end_condition='stop', angle=45, other_parameter=None):
        Command.__init__(self, name='shooter_hood')
        # self.requires(robot.shooter)
        self.robot = robot
        self.button = button
        self.end_condition = end_condition
        self.angle = angle
        self.velocity = 1000  # rpm for the shooter

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp()-self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.robot.enabled_time:2.2f} s **")

        self.robot.shooter.set_flywheel(self.velocity)
        self.robot.shooter.set_hood_setpoint(self.angle)

    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above to tweak speeds and correction
        """
        if self.robot.shooter.hood_controller.atSetpoint():  # probably want to check the flywheel velocity as well
            self.robot.shooter.set_feed_motor(1)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        self.robot.shooter.set_feed_motor(0)
        if self.end_condition == 'stop':
            self.robot.shooter.stop_flywheel(0)
        else:
            pass
        print(f"** {message} {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')

from wpilib.command import Command
from wpilib import Timer, SmartDashboard

class ShooterToggleFlywheel(Command):
    """
    The starts and stops the shooter flywheel.  It's an _instant command_ - it fires and it's done.  no looping.
    Could easily make this a single button toggle or a two button start/stop approach.
    """

    shooter_enabled = False  # use this class variable (not an instance variable)  if you want to make this a toggle

    def __init__(self, robot, omega=-4000):
        Command.__init__(self, name='shooter_flywheel')
        #self.requires(robot.shooter)
        self.robot = robot
        self.omega = omega  # reminder to think in angular velocity at the moment, not linear

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp(), 1)

        print("\n" + f"** Started {self.getName()} with enabling as {not self.shooter_enabled} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.robot.enabled_time:2.2f} s **")
        # toggle button approach
        if self.shooter_enabled:
            self.robot.shooter.stop_flywheel()
            self.shooter_enabled = False
        else:
            self.robot.shooter.set_flywheel(velocity=self.omega)
            self.shooter_enabled = True


    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above to tweak speeds and correction
        """
        pass

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return True

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        # print(f"** {message} {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
        pass  # no need to clutter up the console with the messages

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')

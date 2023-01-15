from wpilib.command import Command
from wpilib import Timer
from wpilib import SmartDashboard
from networktables import NetworkTables

class AutonomousDriveTimed(Command):
    """ This command drives the robot for a fixed amount of time """

    def __init__(self, robot, timeout=None):
        """The constructor"""
        #super().__init__()
        Command.__init__(self, name='auto_drivetimed')
        # Signal that we require ExampleSubsystem
        self.requires(robot.drivetrain)

        if timeout is None:
            self.timeout = 2
        else:
            self.timeout = timeout
        self.setTimeout(self.timeout)

        self.robot = robot

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with timeout {self.timeout} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} with timeout {self.timeout} at {self.start_time} s **")

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.arcade_drive(1,0)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # somehow need to wait for the error level to get to a tolerance... request from drivetrain?
        return self.isTimedOut()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print(f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        self.end(message='Interrupted')


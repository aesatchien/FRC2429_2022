from wpilib.command import Command
from wpilib import Timer

class DriveByJoystick(Command):
    """
    This allows Logitech gamepad to drive the robot. It is always running except when interrupted by another command.
    """

    def __init__(self, robot):
        Command.__init__(self, name='drivebyjoystick')
        self.requires(robot.drivetrain)
        self.robot = robot

        self.max_thrust = 0.6 #.7
        self.max_twist = 0.5 #.6

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        # print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.robot.drivetrain.arcade_drive(-self.max_thrust*self.robot.oi.stick.getRawAxis(1), self.max_twist*self.robot.oi.stick.getRawAxis(4)) #original is 1, 4
        #self.robot.drivetrain.tank_drive_volts(5,5)


    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # we never let this command end - only get interrupted.  it is the default command so it auto-starts when
        # nothing else wants the drivetrain
        return False

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.drive.arcadeDrive(0,0)
        # print(f"** {message} {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **", flush=True)

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')

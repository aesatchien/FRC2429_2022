from wpilib.command import Command
from wpilib import Timer
import math
import networktables

class FRCCharacterization(Command):
    """
    Interact with the frc-characterization tool provided by wpilib.  This is almost all pre-determined.
    Can't actually get this to work since the tool is looking for autonomous and disables settings.  Not sure if i can
    trick it at some point to pull all this out of the main robot.py file.
    """

    entries = []   # data for the frc-configuration tool
    counter = 0
    autoSpeedEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/autospeed")
    telemetryEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/telemetry")
    rotateEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/rotate")

    def __init__(self, robot, button, timeout=60):
        Command.__init__(self, name='frc_characterization')
        self.requires(robot.drivetrain)
        self.robot = robot

        self.button = button
        self.timeout = timeout
        self.setTimeout(self.timeout)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.prior_autospeed = 0
        self.data = ''
        networktables.NetworkTablesInstance.getDefault().setUpdateRate(0.010)
        self.counter = 0

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        # play with the frc characterization tool to see if we can get better values for drivetrain
        now = Timer.getFPGATimestamp()
        left_position = self.robot.drivetrain.l_encoder.getDistance()
        left_rate = self.robot.drivetrain.l_encoder.getRate()
        right_position = self.robot.drivetrain.r_encoder.getDistance()
        right_rate = self.robot.drivetrain.r_encoder.getRate()
        battery = 11
        motor_volts = battery * math.fabs(self.prior_autospeed)
        left_motor_volts = motor_volts
        right_motor_volts = motor_volts
        autospeed = self.autoSpeedEntry.getDouble(0)
        self.prior_autospeed = autospeed

        factor = -1.0 if self.rotateEntry.getBoolean(False) else 1.0
        self.robot.drivetrain.tank_drive_volts(factor * autospeed, -1 * autospeed)
        self.robot.drivetrain.drive.feed()

        vals = [now, battery, autospeed, left_motor_volts, right_motor_volts, left_position, right_position, left_rate,
                right_rate, self.robot.drivetrain.navx.getRotation2d().radians()]
        for i in vals:
            self.entries.append(i)
        self.counter += 1

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # End when we let go of the button
        return not self.button.get()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.drive.arcadeDrive(0,0)
        #print(f"** {message} {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **", flush=True)

        elapsed_time = Timer.getFPGATimestamp() - self.start_time - self.robot.enabled_time
        self.robot.drivetrain.tank_drive_volts(0,0)
        self.data = str(self.entries)
        self.data = self.data[1:-2] + ', '
        self.telemetryEntry.setString(self.data)
        self.entries.clear()
        print(f'** {self.getName()} collected {self.counter} data points in {elapsed_time} s **')
        self.data = ''

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')


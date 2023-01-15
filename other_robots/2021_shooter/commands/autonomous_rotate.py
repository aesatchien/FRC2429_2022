from wpilib.command import Command
from wpilib import Timer
import math
from wpilib import SmartDashboard
from networktables import NetworkTables

class AutonomousRotate(Command):
    """ This command rotates the robot over a given angle with simple proportional + derivative control """
    def __init__(self, robot, setpoint=None, timeout=None, source=None, absolute=False):
        """The constructor"""
        Command.__init__(self, name='auto_rotate')
        self.requires(robot.drivetrain)
        self.setpoint = setpoint
        self.source = source  # sent directly to command or via dashboard
        self.absolute = absolute  # use a relative turn or absolute

        if timeout is None:
            self.timeout = 5
        else:
            self.timeout = timeout
        self.setTimeout(self.timeout)

        self.robot = robot

        self.tolerance = 1
        self.kp = 0.2;  self.kd = 0.1; self.kf = 0.1
        self.start_angle = 0

        self.power = 0; self.max_power = 0.5  # clamp maximum power for turning so we don't over turn
        self.error = 0;  self.prev_error = 0

    def initialize(self):
        """Called just before this Command runs the first time."""

        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} with setpoint {self.setpoint} at {self.start_time} s **")

        if self.source == 'dashboard':
            self.setpoint = SmartDashboard.getNumber('z_angle', 1)
        # may want to break if no valid setpoint is passed

        self.start_angle = self.robot.drivetrain.navx.getAngle()
        if self.absolute:  # trust the navx to be correctly oriented to the field
            self.setpoint = self.setpoint - self.start_angle
            if self.setpoint > 180:
                self.setpoint = -(360 - self.setpoint)
            if self.setpoint < -180:
                self.setpoint = (360 + self.setpoint)

        self.error = 0
        self.prev_error = 0

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        self.error = self.setpoint - (self.robot.drivetrain.navx.getAngle()-self.start_angle)
        self.power = self.kp * self.error + self.kf * math.copysign(1, self.error) + self.kd * (self.error - self.prev_error) / 0.02
        self.prev_error = self.error
        if self.power >0:
            self.power = min(self.max_power, self.power)
        else:
            self.power = max(-self.max_power, self.power)

        self.robot.drivetrain.arcade_drive(thrust=0, twist=self.power)
        #SmartDashboard.putNumber("error", self.error)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        # I know I could do this with a math.copysign, but this is more readable
        if self.setpoint > 0:
            return self.error <= self.tolerance or self.isTimedOut()
        else:
            return self.error >= -self.tolerance or self.isTimedOut()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print(f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time - self.start_time, 1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')

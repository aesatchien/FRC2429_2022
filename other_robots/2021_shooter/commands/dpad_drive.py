from wpilib.command import Command
from wpilib import Timer, SmartDashboard
import math

class DpadDrive(Command):
    """
    This allows Logitech gamepad's dpad to drive the robot. It overrides the stick.
    Change the drive_power and twist_power variables to change how it reacts
    """

    def __init__(self, robot, button):
        Command.__init__(self, name='DpadDrive')
        self.requires(robot.drivetrain)
        self.robot = robot
        self.button = button
        self.mode = None
        self.scale = 0.3
        self.twist_scale = 0.35

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp(), 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.robot.enabled_time:2.2f} s **")
        #self.heading = self.robot.navigation.get_angle()

    def execute(self):
        """
        Called repeatedly when this Command is scheduled to run
        Should not have to change this if it works - just change variables above to tweak speeds and correction
        """
        # easy to correct for heading drift - we know we're trying to drive straight if we keep previous angle heading ...
        angle = self.button.angle() * math.pi / 180.
        thrust, twist = self.scale*math.cos(angle), self.twist_scale*math.sin(angle)


        if self.robot.isReal():
            if self.mode == 'velocity':
                self.robot.drivetrain.mecanum_velocity_cartesian(thrust=thrust, strafe=0, z_rotation=twist)
            else:
                #self.robot.drivetrain.smooth_drive(thrust=thrust, strafe=0, twist=twist)
                self.robot.drivetrain.arcade_drive(thrust=thrust, twist=twist)
        else:
            # simulation
            self.robot.drivetrain.arcade_drive(thrust, twist)

    def isFinished(self):
        """Make this return true when this Command no longer needs to run execute()"""
        return not self.button.get()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        self.robot.drivetrain.stop()
        print(f"** {message} {self.getName()} at {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)} s **")
    def interrupted(self):
        """Called when another command which requires one or more of the same subsystems is scheduled to run."""
        self.end(message='Interrupted')

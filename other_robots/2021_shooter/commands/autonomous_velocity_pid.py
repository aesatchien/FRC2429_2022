
# Note - this is just for playing around - not the the feed forward works this is fine

from wpilib.command import Command
from wpimath import controller
import wpimath.kinematics
from wpilib import Timer, SmartDashboard

import subsystems.drive_constants as drive_constants

class AutonomousVelocityPID(Command):
    """Testing the velocity controllers before constructing the ramsete command """
    def __init__(self, robot, left_speed_setpoint=1.0, right_speed_setpoint=-1, timeout=6):
        Command.__init__(self, name='auto_velocity')
        self.robot = robot
        self.requires(robot.drivetrain)
        self.setTimeout(timeout)
        self.use_dash = True

        # initialize the feed forward so we can use velocities
        self.feed_forward = wpilib.controller.SimpleMotorFeedforwardMeters(
            drive_constants.ks_volts, drive_constants.kv_volt_seconds_per_meter, drive_constants.ka_volt_seconds_squared_per_meter)

        self.kp = 0.0
        self.kd = 0.0
        self.left_speed_setpoint = left_speed_setpoint
        self.right_speed_setpoint = right_speed_setpoint

        if self.use_dash:
            SmartDashboard.putNumber('l_vel', self.left_speed_setpoint)
            SmartDashboard.putNumber('r_vel', self.right_speed_setpoint)
            SmartDashboard.putNumber('kp_vel', self.kp)
            SmartDashboard.putNumber('kd_vel', self.kd)

    def initialize(self):
        """Called just before this Command runs the first time."""
        self.start_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **")
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time} s **")

        if self.use_dash:
            self.kp = SmartDashboard.getNumber('kp_vel', self.kp)
            self.kd = SmartDashboard.getNumber('kd_vel', self.kd)
            self.left_speed_setpoint = SmartDashboard.getNumber('l_vel', self.left_speed_setpoint)
            self.right_speed_setpoint = SmartDashboard.getNumber('r_vel', self.right_speed_setpoint)

        self.left_controller = wpilib.controller.PIDController(self.kp, 0 , self.kd, period=0.02)
        self.right_controller = wpilib.controller.PIDController(self.kp, 0 , self.kd, period=0.02)

        self.previous_time = -1

        self.left_controller.reset()
        self.right_controller.reset()

    def execute(self) -> None:
        current_time = self.timeSinceInitialized()
        dt = current_time - self.previous_time

        if self.previous_time < 0:
            self.robot.drivetrain.tank_drive_volts(0, 0)
            self.previous_time = current_time
            return

        left_feed_forward = self.feed_forward.calculate(self.left_speed_setpoint)
        right_feed_forward = self.feed_forward.calculate(self.right_speed_setpoint)

        ws_left, ws_right = self.robot.drivetrain.get_rate(self.robot.drivetrain.l_encoder), self.robot.drivetrain.get_rate(self.robot.drivetrain.r_encoder)
        left_output = left_feed_forward + self.left_controller.calculate(ws_left, self.left_speed_setpoint)
        right_output = right_feed_forward + self.left_controller.calculate(ws_right, self.right_speed_setpoint)

        self.robot.drivetrain.tank_drive_volts(left_output, right_output)
        self.robot.drivetrain.drive.feed()

    def isFinished(self) -> bool:
        return self.isTimedOut()

    def end(self, message='Ended'):
        """Called once after isFinished returns true"""
        end_time = round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)
        print(f"** {message} {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        SmartDashboard.putString("alert", f"** Ended {self.getName()} at {end_time} s after {round(end_time-self.start_time,1)} s **")
        self.robot.drivetrain.stop()

    def interrupted(self):
        self.end(message='Interrupted')

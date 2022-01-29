import time
from commands2 import RunCommand, RamseteCommand
from commands2.button import JoystickButton, Button

from wpilib import XboxController, SmartDashboard, SendableChooser
from wpilib.controller import RamseteController, PIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from subsystems.drivetrain import Drivetrain
from commands.autonomous_ramsete import AutonomousRamsete

import constants
import trajectory_io

class RobotContainer:

    """
    This class hosts the bulk of the robot's functions. Little robot logic needs to be
    handled here or in the robot periodic methods, as this is a command-based system.
    The structure (commands, subsystems, and button mappings) should be done here.
    """

    def __init__(self):

        self.start_time = time.time()

        # Create an instance of the drivetrain subsystem.
        self.robot_drive = Drivetrain()

        # Create the driver's controller.
        self.driver_controller = XboxController(constants.k_driver_controller_port)
        # Configure and set the button bindings for the driver's controller.
        self.initialize_joysticks()
        self.initialize_dashboard()

        # Set the default command for the drive subsystem. It allows the robot to drive with the controller.
        #TODO: set different twist multipliers when stopped for high and low gear for consistent turning performance, reduce acceleration limit: motors stutter in high gear when at full throttle from stop
        self.robot_drive.setDefaultCommand(
            RunCommand(
                lambda: self.robot_drive.arcade_drive(-1*constants.k_thrust_scale*self.driver_controller.getRawAxis(constants.k_controller_thrust_axis),
                                                      constants.k_twist_scale*self.driver_controller.getRawAxis(constants.k_controller_twist_axis), ),
                self.robot_drive,)
        )

        if False:  # test tank drive
            self.robot_drive.setDefaultCommand(
                RunCommand(
                    lambda: self.robot_drive.tank_drive_volts(-self.driver_controller.getRawAxis(constants.k_controller_thrust_axis) * 12,
                                                          self.driver_controller.getRawAxis(constants.k_controller_twist_axis) * 12, ),
                    self.robot_drive,)
            )

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def get_autonomous_command(self):
        return AutonomousRamsete(container=self, drive=self.robot_drive)  # .andThen(lambda: self.robot_drive.tank_drive_volts(0, 0))

    # --------------  OI  ---------------

    def initialize_joysticks(self):
        """Configure the buttons for the driver's controller"""

        self.buttonA = JoystickButton(self.driver_controller, 1)
        self.buttonB = JoystickButton(self.driver_controller, 2)
        self.buttonX = JoystickButton(self.driver_controller, 3)
        self.buttonY = JoystickButton(self.driver_controller, 4)
        self.buttonLB = JoystickButton(self.driver_controller, 5)
        self.buttonRB = JoystickButton(self.driver_controller, 6)
        self.buttonBack = JoystickButton(self.driver_controller, 7)
        self.buttonStart = JoystickButton(self.driver_controller, 8)
        #self.axisButtonLT = AxisButton(self.driver_controller, 2)
        #self.axisButtonRT = AxisButton(self.driver_controller, 3)

        self.buttonA.whenPressed(AutonomousRamsete(container=self, drive=self.robot_drive))

        # We won't do anything with this button itself, so we don't need to define a variable.
        (
            self.buttonRB
            .whenPressed(lambda: self.robot_drive.set_max_output(0.15))
            .whenReleased(lambda: self.robot_drive.set_max_output(1))
        )

    def initialize_dashboard(self):
        self.path_chooser = SendableChooser()
        SmartDashboard.putData('ramsete path', self.path_chooser)
        choices = trajectory_io.get_pathweaver_paths() + ['z_loop', 'z_poses', 'z_points', 'z_test']
        for ix, position in enumerate(choices):
            if ix == 0:
                self.path_chooser.setDefaultOption(position, position)
            else:
                self.path_chooser.addOption(position, position)

        self.velocity_chooser = SendableChooser()
        SmartDashboard.putData('path velocity', self.velocity_chooser)
        velocities = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]
        for ix, velocity in enumerate(velocities):
            if ix == 4: # 2.5 will be the default
                self.velocity_chooser.setDefaultOption(str(velocity), velocity)
            else:
                self.velocity_chooser.addOption(str(velocity), velocity)
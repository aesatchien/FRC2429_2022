import time
from commands2 import RunCommand, RamseteCommand, ConditionalCommand, Trigger
from commands2.button import JoystickButton, Button, POVButton
from wpilib import XboxController, SmartDashboard, SendableChooser, Joystick

from subsystems.drivetrain import Drivetrain

import constants

class RobotContainer:

    """
    This class hosts the bulk of the robot's functions. Little robot logic needs to be
    handled here or in the robot periodic methods, as this is a command-based system.
    The structure (commands, subsystems, and button mappings) should be done here.
    """

    def __init__(self):

        self.start_time = time.time()

        self.competition_mode = True  # set up second controller

        # Create an instance of the drivetrain subsystem.
        self.robot_drive = Drivetrain()

        # Create the driver's controller
        self.driver_controller = None
        self.co_driver_controller = None
        # Configure and set the button bindings for the driver's controller.
        self.initialize_joysticks()

        # Set the default command for the drive subsystem. It allows the robot to drive with the controller.
        #TODO: set different twist multipliers when stopped for high and low gear for consistent turning performance, reduce acceleration limit: motors stutter in high gear when at full throttle from stop
        if False:
            self.robot_drive.setDefaultCommand(DriveByJoytick(self, self.robot_drive, control_type='velocity', scaling=1.0))

        if False:
            # test arcade drive
            self.robot_drive.setDefaultCommand(
                RunCommand(
                    lambda: self.robot_drive.arcade_drive(
                        -1 * constants.k_thrust_scale * self.driver_controller.getRawAxis(
                            constants.k_controller_thrust_axis),
                        constants.k_twist_scale * self.driver_controller.getRawAxis(
                            constants.k_controller_twist_axis), ),
                    self.robot_drive, ))

            # test tank drive
            self.robot_drive.setDefaultCommand(
                RunCommand(
                    lambda: self.robot_drive.tank_drive_volts(-self.driver_controller.getRawAxis(constants.k_controller_thrust_axis) * 12,
                                                          self.driver_controller.getRawAxis(constants.k_controller_twist_axis) * 12, ),
                    self.robot_drive,))

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def get_autonomous_command(self):
        return self.autonomous_chooser.getSelected()

    # --------------  OI  ---------------

    def initialize_joysticks(self):
        """Configure the buttons for the driver's controller"""

        # Create the driver's controller.
        self.driver_controller = Joystick(constants.k_driver_controller_port)
        self.buttonA = JoystickButton(self.driver_controller, 1)
        self.buttonB = JoystickButton(self.driver_controller, 2)
        self.buttonX = JoystickButton(self.driver_controller, 3)
        self.buttonY = JoystickButton(self.driver_controller, 4)
        self.buttonLB = JoystickButton(self.driver_controller, 5)
        self.buttonRB = JoystickButton(self.driver_controller, 6)
        self.buttonBack = JoystickButton(self.driver_controller, 7)
        self.buttonStart = JoystickButton(self.driver_controller, 8)
        self.buttonUp = POVButton(self.driver_controller, 0)
        self.buttonDown = POVButton(self.driver_controller, 180)
        self.buttonLeft = POVButton(self.driver_controller, 270)
        self.buttonRight = POVButton(self.driver_controller, 90)
        #self.buttonLeftAxis = AxisButton(self.driver_controller, 1)
        #self.buttonRightAxis = AxisButton(self.driver_controller, 5)

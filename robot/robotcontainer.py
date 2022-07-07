import time
from commands2 import InstantCommand
from commands2.button import JoystickButton, POVButton
from wpilib import XboxController, SmartDashboard, SendableChooser, AddressableLED

from subsystems.drivetrain import Drivetrain

from subsystems.vision import Vision
from subsystems.led import Led

from commands.drive_by_joystick import DriveByJoytick
from commands.tune_sparkmax_drive import TuneSparkmax
from commands.led_loop import LedLoop

from trigger.axis_button import AxisButton

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

        self.competition_mode = False  # set up second controller

        # Create an instance of the drivetrain subsystem.
        self.robot_drive = Drivetrain()

        self.robot_vision = Vision()
        self.robot_led = Led()

        # Create the driver's controller
        self.driver_controller = None
        self.co_driver_controller = None
        # Configure and set the button bindings for the driver's controller.
        self.initialize_joysticks()
        self.initialize_dashboard()

        # Set the default command for the LED strip
        # depends on other subsystems being present
        # self.robot_led.setDefaultCommand(LedLoop(self))

        # Set the default command for the drive subsystem. It allows the robot to drive with the controller.
        #TODO: set different twist multipliers when stopped for high and low gear for consistent turning performance, reduce acceleration limit: motors stutter in high gear when at full throttle from stop
        if not constants.k_is_simulation:
            self.robot_drive.setDefaultCommand(DriveByJoytick(self, self.robot_drive, control_type='velocity', scaling=1.0))
        else:
            self.robot_drive.setDefaultCommand(
                DriveByJoytick(self, self.robot_drive, control_type='simulation', scaling=1.0))

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
        self.driver_controller = XboxController(constants.k_driver_controller_port)
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
        self.buttonLeftAxis = AxisButton(self.driver_controller, 2)
        self.buttonRightAxis = AxisButton(self.driver_controller, 3)

        if self.competition_mode:
            self.co_driver_controller = XboxController(constants.k_co_driver_controller_port)
            self.co_buttonB = JoystickButton(self.co_driver_controller, 2)
            self.co_buttonA = JoystickButton(self.co_driver_controller, 1)
            self.co_buttonY = JoystickButton(self.co_driver_controller, 4)
            self.co_buttonX = JoystickButton(self.co_driver_controller, 3)
            self.co_buttonLB = JoystickButton(self.co_driver_controller, 5)
            self.co_buttonRB = JoystickButton(self.co_driver_controller, 6)
            self.co_buttonBack = JoystickButton(self.co_driver_controller, 7)
            self.co_buttonStart = JoystickButton(self.co_driver_controller, 8)
            self.co_buttonUp = POVButton(self.co_driver_controller, 0)
            self.co_buttonDown = POVButton(self.co_driver_controller, 180)
            self.co_buttonLeft = POVButton(self.co_driver_controller, 270)
            self.co_buttonRight = POVButton(self.co_driver_controller, 90)
            self.co_leftTrigger = AxisButton(self.co_driver_controller, 2)
            self.co_rightTrigger = AxisButton(self.co_driver_controller, 3)
        else:
            self.co_driver_controller = None

        # lots of putDatas for testing on the dash
        SmartDashboard.putData(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='position', spin=False))
        SmartDashboard.putData('Reset Encoders', InstantCommand(lambda: self.robot_drive.reset_encoders()))
        SmartDashboard.putData('Reset NavX', InstantCommand(lambda: self.robot_drive.navx.setAngleAdjustment(-constants.k_start_heading - self.robot_drive.navx.getYaw())))

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

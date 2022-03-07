import time
from commands2 import RunCommand, RamseteCommand, ConditionalCommand, Trigger
from commands2.button import JoystickButton, Button, POVButton
from wpilib import XboxController, SmartDashboard, SendableChooser, Joystick

from subsystems.drivetrain import Drivetrain
from subsystems.intake import Intake
from subsystems.shooter import Shooter
from subsystems.climber import Climber
from subsystems.pneumatics import Pneumatics
from subsystems.indexer import Indexer
from subsystems.vision import Vision

from commands.autonomous_ramsete import AutonomousRamsete
from commands.auto_ramsete_wpilib import AutoRamseteWpilib
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.toggle_shooter import ToggleShooter
from commands.toggle_feed import ToggleFeed
from commands.toggle_compressor import ToggleCompressor
from commands.spin_climber import SpinClimber
from commands.toggle_shifting import ToggleShifting
from commands.toggle_intake import ToggleIntake
from commands.timed_feed import TimedFeed
from commands.auto_fetch_ball import AutoFetchBall
from commands.tune_sparkmax_drive import TuneSparkmax
from commands.auto_rotate_sparkmax import AutoRotateSparkmax
from commands.auto_rotate_imu import AutoRotateImu
from commands.autonomous_lower_group import AutonomousLowerGroup
from commands.drive_by_joystick import DriveByJoytick
from commands.hold_feed import HoldFeed

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

        self.competition_mode = True  # set up second controller

        # Create an instance of the drivetrain subsystem.
        self.robot_drive = Drivetrain()
        self.robot_intake = Intake()
        self.robot_shooter = Shooter()
        self.robot_pneumatics = Pneumatics()
        self.robot_climber = Climber()
        self.robot_indexer = Indexer()
        self.robot_vision = Vision()

        # Create the driver's controller
        self.driver_controller = None
        self.co_driver_controller = None
        # Configure and set the button bindings for the driver's controller.
        self.initialize_joysticks()
        self.initialize_dashboard()
        
        #mode
        self.is_endgame = False

        # Set the default command for the drive subsystem. It allows the robot to drive with the controller.
        #TODO: set different twist multipliers when stopped for high and low gear for consistent turning performance, reduce acceleration limit: motors stutter in high gear when at full throttle from stop
        if not constants.k_is_simulation:
            self.robot_drive.setDefaultCommand(DriveByJoytick(self, self.robot_drive, control_type='velocity'))
        else:
            self.robot_drive.setDefaultCommand(
                RunCommand(
                    lambda: self.robot_drive.arcade_drive(
                        -1 * constants.k_thrust_scale * self.driver_controller.getRawAxis(
                            constants.k_controller_thrust_axis),
                        constants.k_twist_scale * self.driver_controller.getRawAxis(
                            constants.k_controller_twist_axis), ),
                    self.robot_drive, ))

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
        return AutonomousRamsete(container=self, drive=self.robot_drive)  # .andThen(lambda: self.robot_drive.tank_drive_volts(0, 0))

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

        #climbing
        self.buttonRight.whenHeld(SpinClimber(self, self.robot_climber))

        #shooting
        #todo: aim assist self.buttonA.whenHeld(AIM ASSIST)
        self.buttonB.whenPressed(ToggleShooter(self, self.robot_shooter, 2000))

        #pneumatics
        self.buttonBack.whenPressed(ToggleShifting(self, self.robot_pneumatics))
        self.buttonStart.whenPressed(ToggleCompressor(self, self.robot_pneumatics))
        self.buttonLB.whenPressed(lambda: self.robot_pneumatics.pp_short())
        self.buttonRB.whenPressed(lambda: self.robot_pneumatics.pp_long())

        #climbing
        # todo: reset to horizontal self.buttonRight.whenPressed(reset to horizontl)

        #intake
        self.buttonDown.whenPressed(ToggleIntake(self, self.robot_pneumatics))
        #self.buttonDown.whenPressed(TimedFeed(self, self.robot_indexer, 2, 5))
        self.buttonUp.whileHeld(HoldFeed(self, self.robot_indexer, 3))

        
        #vision
        self.buttonA.whileHeld(AutoFetchBall(self, self.robot_drive, self.robot_vision))
        # Testing autonomous calls - may want to bind them to calling on the dashboard
         #self.buttonA.whenPressed(AutonomousRamsete(container=self, drive=self.robot_drive))
        #self.buttonRight.whenPressed(AutonomousRamsete(container=self, drive=self.robot_drive, source='pathweaver'))
        # self.buttonLeft.whenPressed(AutonomousRamsete(container=self, drive=self.robot_drive, source='waypoint'))

        #SmartDashboard.putData(AutonomousRamsete(container=self, drive=self.robot_drive, source='dash'))
        #SmartDashboard.putData(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='velocity', spin=False))
        # self.buttonX.whenPressed(AutoRotateImu(self, self.robot_drive, 30))
        # self.buttonX.whenPressed(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='position', spin=False))
        SmartDashboard.putData(AutoFetchBall(self, self.robot_drive, self.robot_vision))

        SmartDashboard.putNumber('/AutoFetchBall/kp', 0)
        SmartDashboard.putNumber('/AutoFetchBall/kd', 0)
        SmartDashboard.putNumber('/AutoFetchBall/kf', 0)

        #self.buttonX.whenPressed(AutoRotateImu(container=self, drive=self.robot_drive, degrees=90).withTimeout(2))
        self.buttonX.whileHeld(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='velocity', spin=False))

        #SmartDashboard.putData(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='position', spin=False))
        #SmartDashboard.putData(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='velocity', spin=False))
        #self.buttonX.whenPressed(AutoRotateSparkmax(self, self.robot_drive, 30))
        # self.buttonX.whenPressed(AutoRotateImu(container=self, drive=self.robot_drive, degrees=90))

        SmartDashboard.putData(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='velocity', spin=False))
        SmartDashboard.putData(AutonomousLowerGroup(container=self))
        SmartDashboard.putData(AutonomousRamsete(container=self, drive=self.robot_drive, source='dash'))
        SmartDashboard.putData(IntakeMotorToggle(container=self, intake=self.robot_intake, velocity=0.65, source='dash'))
        SmartDashboard.putData(ToggleIntake(self, self.robot_pneumatics))

        if self.competition_mode:
            #climber
            #self.co_buttonY.whileHeld(SpinClimber(self, self.robot_climber))

            #intake
            self.co_buttonDown.whenPressed(ToggleIntake(self, self.robot_pneumatics))
            self.co_buttonLB.whenPressed(IntakeMotorToggle(self, self.robot_intake, 0.65))

            #indexer
            self.co_buttonRB.whenPressed(ToggleFeed(self, self.robot_indexer, 2))
            # self.co_buttonRB.whenPressed(TimedFeed(self, self.robot_indexer, 3, 4))
            self.co_buttonUp.whileHeld(HoldFeed(self, self.robot_indexer, 3))


            #shooter
            self.co_buttonA.whenPressed(ToggleShooter(self, self.robot_shooter, 2000))

            #compressor
            self.co_buttonStart.whenPressed(ToggleCompressor(self, self.robot_pneumatics))

        # We won't do anything with this button itself, so we don't need to define a variable.
        (
            self.buttonRB
            .whenPressed(lambda: self.robot_drive.set_max_output(0.25))
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
import time
from commands2 import InstantCommand
from commands2.button import JoystickButton, POVButton
from wpilib import XboxController, SmartDashboard, SendableChooser

from subsystems.drivetrain import Drivetrain
from subsystems.intake import Intake
from subsystems.shooter import Shooter
from subsystems.climber import Climber
from subsystems.pneumatics import Pneumatics
from subsystems.indexer import Indexer
from subsystems.vision import Vision
#from subsystems.led import Led

from commands.auto_set_pose import AutoSetPose
from commands.auto_shoot import AutoShoot
from commands.auto_pickup import AutoPickup
from commands.auto_track_hub import AutoTrackHub
from commands.auto_rotate_sparkmax import AutoRotateSparkmax
from commands.auto_rotate_imu import AutoRotateImu
from commands.auto_fetch_ball import AutoFetchBall
from commands.auto_ramsete import AutoRamsete
from commands.drive_by_joystick import DriveByJoytick
from commands.intake_motor_toggle import IntakeMotorToggle
from commands.indexer_hold import IndexerHold
from commands.indexer_toggle import IndexerToggle
from commands.compressor_toggle import CompressorToggle
from commands.climber_spin import ClimberSpin
from commands.intake_position_toggle import IntakePositionToggle
from commands.shifter_toggle import ShifterToggle
from commands.shooter_toggle import ShooterToggle
from commands.shooter_hood_toggle import ShooterHoodToggle
from commands.tune_sparkmax_drive import TuneSparkmax
from commands.tune_sparkmax_climber import TuneSparkmaxClimber
from commands.climber_rotate_set_distance import ClimberRotateSetDistance
from commands.led_loop import LedLoop

from commands.autonomous_two_ball import AutonomousTwoBall
from commands.autonomous_general_two_ball import AutonomousGeneralTwoBall
from commands.autonomous_stage_two import AutonomousStageTwo
from commands.autonomous_three_ball import AutonomousThreeBall
from commands.autonomous_four_ball import AutonomousFourBall

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

        self.competition_mode = True  # set up second controller

        # Create an instance of the drivetrain subsystem.
        self.robot_drive = Drivetrain()
        self.robot_intake = Intake()
        self.robot_shooter = Shooter()
        self.robot_pneumatics = Pneumatics()
        self.robot_climber = Climber()
        self.robot_indexer = Indexer()
        self.robot_vision = Vision()
        # self.robot_led = Led()

        # Create the driver's controller
        self.driver_controller = None
        self.co_driver_controller = None
        # Configure and set the button bindings for the driver's controller.
        self.initialize_joysticks()
        self.initialize_dashboard()

        # Set the default command for the LED strip
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

        #climbing
        self.buttonRight.whenHeld(ClimberSpin(self, self.robot_climber))

        #shooting
        self.buttonA.whileHeld(AutoTrackHub(self, self.robot_drive, self.robot_shooter, self.robot_vision, self.robot_pneumatics))
        self.buttonB.whileHeld(AutoFetchBall(self, self.robot_drive, self.robot_vision))

        #pneumatics
        self.buttonStart.whenPressed(ShifterToggle(self, self.robot_pneumatics))
        self.buttonBack.whenPressed(CompressorToggle(self, self.robot_pneumatics))

        self.buttonLB.whenPressed(lambda: self.robot_pneumatics.pp_short())
        self.buttonRB.whenPressed(lambda: self.robot_pneumatics.pp_long())

        #intake
        self.buttonDown.whenPressed(IntakePositionToggle(self, self.robot_pneumatics))
        self.buttonUp.whileHeld(IndexerHold(self, self.robot_indexer, 3))

        if self.competition_mode:
            #climber
            self.co_buttonB.whenPressed(ClimberRotateSetDistance(self, self.robot_climber, 36))
            self.co_buttonY.whenPressed(ClimberRotateSetDistance(self, self.robot_climber, 126))

            #intake
            self.co_buttonDown.whenPressed(IntakePositionToggle(self, self.robot_pneumatics))
            self.co_buttonLB.whenPressed(IntakeMotorToggle(self, self.robot_intake, 0.7))

            #indexer
            self.co_buttonRB.whenPressed(IndexerToggle(self, self.robot_indexer, 6))
            self.co_buttonUp.whileHeld(IndexerHold(self, self.robot_indexer, 5))

            #shooter
            self.co_buttonA.whenPressed(ShooterToggle(self, self.robot_shooter, 2500))
            self.co_rightTrigger.whenPressed(lambda: self.robot_pneumatics.set_shooter_hood_position(position='extend'))
            self.co_leftTrigger.whenPressed(lambda: self.robot_pneumatics.set_shooter_hood_position(position='retract'))

            #compressor
            self.co_buttonBack.whenPressed(CompressorToggle(self, self.robot_pneumatics))

        # lots of putdatas for testing on the dash
        SmartDashboard.putData(TuneSparkmax(container=self, drive=self.robot_drive, setpoint=1, control_type='position', spin=False))
        SmartDashboard.putData(TuneSparkmaxClimber(container=self, climber=self.robot_climber, setpoint=1, control_type='position'))

        SmartDashboard.putData(AutoSetPose(self, pose=None))
        SmartDashboard.putData(AutonomousTwoBall(container=self))
        SmartDashboard.putData(AutoRamsete(container=self, drive=self.robot_drive, source='pathweaver'))
        SmartDashboard.putData(CompressorToggle(self, self.robot_pneumatics))
        SmartDashboard.putData(IntakeMotorToggle(container=self, intake=self.robot_intake, velocity=0.5, source='dash')) # was 0.85
        SmartDashboard.putData(IntakePositionToggle(self, self.robot_pneumatics))
        SmartDashboard.putData(IndexerHold(self, self.robot_indexer, 2.5))  # was 3
        SmartDashboard.putData(ShooterToggle(self, self.robot_shooter, 3000))
        SmartDashboard.putData(ShooterHoodToggle(self, self.robot_pneumatics))
        SmartDashboard.putData('Reset Encoders', InstantCommand(lambda: self.robot_drive.reset_encoders()))
        SmartDashboard.putData('Reset NavX', InstantCommand(lambda: self.robot_drive.navx.setAngleAdjustment(-constants.k_start_heading - self.robot_drive.navx.getYaw())))

        SmartDashboard.putData(AutoRotateSparkmax(self, self.robot_drive, target='degrees', degrees=90))
        SmartDashboard.putData(AutoFetchBall(self, self.robot_drive, self.robot_vision))

        SmartDashboard.putData(AutoRotateImu(self, self.robot_drive, source='degrees', degrees=90))

        SmartDashboard.putData(AutoShoot(self))
        SmartDashboard.putData(AutoPickup(self))

        # We won't do anything with this button itself, so we don't need to define a variable.
        # self.buttonRB.whenPressed(lambda: self.robot_drive.set_max_output(0.25)).whenReleased(lambda: self.robot_drive.set_max_output(1))

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

        # populate autonomous routines
        self.autonomous_chooser = SendableChooser()
        SmartDashboard.putData('autonomous routines', self.autonomous_chooser)
        self.autonomous_chooser.setDefaultOption('2 ball only', AutonomousTwoBall(self))
        self.autonomous_chooser.addOption('2 ball general', AutonomousGeneralTwoBall(self))
        self.autonomous_chooser.addOption('3 ball terminal', AutonomousThreeBall(self))
        self.autonomous_chooser.addOption('4 ball lower', AutonomousFourBall(self))
        self.autonomous_chooser.addOption("Ramsete Test", AutoRamsete(container=self, drive=self.robot_drive, dash=False, relative=False, source='pathweaver'))

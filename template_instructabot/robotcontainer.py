import time
from commands2 import InstantCommand
from commands2.button import JoystickButton, POVButton
from wpilib import XboxController, SmartDashboard, SendableChooser

from subsystems.pneumatics import Pneumatics

from commands.compressor_toggle import CompressorToggle
from commands.pneumatics_small_piston_toggle import SmallPistonToggle

import constants

class RobotContainer:

    """
    This class hosts the bulk of the robot's functions. Little robot logic needs to be
    handled here or in the robot periodic methods, as this is a command-based system.
    The structure (commands, subsystems, and button mappings) should be done here.
    """

    def __init__(self):

        self.start_time = time.time()

        self.competition_mode = False  # set up second controller

        # Create an instance of the appropriate subsystems.
        self.robot_pneumatics = Pneumatics()

        # Create the driver's controller
        self.driver_controller = None
        self.co_driver_controller = None
        # Configure and set the button bindings for the driver's controller.
        self.initialize_joysticks()
        self.initialize_dashboard()

        # Set the default command for the LED strip
        # self.robot_led.setDefaultCommand(LedLoop(self))

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
        # self.buttonLB = JoystickButton(self.driver_controller, 5)
        # self.buttonRB = JoystickButton(self.driver_controller, 6)
        self.buttonBack = JoystickButton(self.driver_controller, 7)
        # self.buttonStart = JoystickButton(self.driver_controller, 8)
        # self.buttonUp = POVButton(self.driver_controller, 0)
        # self.buttonDown = POVButton(self.driver_controller, 180)
        # self.buttonLeft = POVButton(self.driver_controller, 270)
        # self.buttonRight = POVButton(self.driver_controller, 90)
        # self.buttonLeftAxis = AxisButton(self.driver_controller, 2)
        # self.buttonRightAxis = AxisButton(self.driver_controller, 3)

        #climbing
        #self.buttonRight.whenHeld(ClimberSpin(self, self.robot_climber))

        #shooting
        #self.buttonA.whileHeld(AutoTrackHub(self, self.robot_drive, self.robot_shooter, self.robot_vision, self.robot_pneumatics))
        #self.buttonB.whileHeld(AutoFetchBall(self, self.robot_drive, self.robot_vision))

        #pneumatics
        self.buttonBack.whenPressed(CompressorToggle(self, self.robot_pneumatics))
        #self.buttonA.whenPressed(lambda: self.robot_pneumatics.toggle_intake())

        self.buttonA.whenPressed(lambda: self.robot_pneumatics.set_small_piston_position(position='extend'))
        self.buttonB.whenPressed(lambda: self.robot_pneumatics.set_small_piston_position(position='retract'))

        self.buttonX.whenPressed(lambda: self.robot_pneumatics.set_large_piston_position(position='extend'))
        self.buttonY.whenPressed(lambda: self.robot_pneumatics.set_large_piston_position(position='retract'))

        #self.buttonRB.whenPressed(lambda: self.robot_pneumatics.pp_long())

        #intake
        #self.buttonDown.whenPressed(IntakePositionToggle(self, self.robot_pneumatics))
        #self.buttonUp.whileHeld(IndexerHold(self, self.robot_indexer, 3))

        # lots of putdatas for testing on the dash
        SmartDashboard.putData(CompressorToggle(self, self.robot_pneumatics))
        SmartDashboard.putData(ShooterHoodToggle(self, self.robot_pneumatics))

        # We won't do anything with this button itself, so we don't need to define a variable.
        # self.buttonRB.whenPressed(lambda: self.robot_drive.set_max_output(0.25)).whenReleased(lambda: self.robot_drive.set_max_output(1))

    def initialize_dashboard(self):

        self.path_chooser = SendableChooser()
        SmartDashboard.putData('ramsete path', self.path_chooser)
        # choices = trajectory_io.get_pathweaver_paths() + ['z_loop', 'z_poses', 'z_points', 'z_test']
        # for ix, position in enumerate(choices):
        #     if ix == 0:
        #         self.path_chooser.setDefaultOption(position, position)
        #     else:
        #         self.path_chooser.addOption(position, position)

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
        #self.autonomous_chooser.setDefaultOption('2 ball only', AutonomousTwoBall(self))
        #self.autonomous_chooser.addOption('2 ball general', AutonomousGeneralTwoBall(self))
        #self.autonomous_chooser.addOption('3 ball terminal', AutonomousThreeBall(self))
        #self.autonomous_chooser.addOption('4 ball lower', AutonomousFourBall(self))
        #self.autonomous_chooser.addOption("Ramsete Test", AutoRamsete(container=self, drive=self.robot_drive, dash=False, relative=False, source='pathweaver'))

# OI compatible with robotpy 2020
import wpilib
from wpilib import SmartDashboard, SendableChooser
# from wpilib.command import JoystickButton, POVButton
from wpilib.buttons import JoystickButton, POVButton
from triggers.dpad import Dpad
from triggers.axis_button import AxisButton

# commands to bind
from commands.dpad_drive import DpadDrive
from commands.autonomous_rotate import AutonomousRotate
from commands.autonomous_drive_timed import AutonomousDriveTimed
from commands.autonomous_ramsete import AutonomousRamsete
from commands.autonomous_home_slalom import AutonomousSlalom
from commands.autonomous_home_bounce import AutonomousBounce
from commands.autonomous_drive_pid import AutonomousDrivePID
from commands.autonomous_velocity_pid import AutonomousVelocityPID
from commands.shooter_toggle_flywheel import ShooterToggleFlywheel
from commands.shooter_hood import ShooterHood
from commands.shooter_fire import ShooterFire
from commands.shooter_feed import ShooterFeed
from commands.drive_to_ball import DriveToBall

import subsystems.drive_constants as drive_constants

class OI(object):
    """
    The operator interface of the robot.  Note we use competition_mode to determine if we will
    initialize a second joystick.  Apparently 2021 added something so we don't have to do this anymore...
    """
    def __init__(self, robot):
        self.robot = robot

        # Set single or double joystick mode
        self.competition_mode = False

        self.initialize_joystics()
        self.assign_buttons()

        self.initialize_dashboard()

    def assign_buttons(self):
        """Assign commands to buttons here"""
        # *** NOTE - THESE CAN FAIL IN COMPETITION IF YOU ARE RELYING ON A BUTTON TO BE HELD DOWN! ***
        self.dpad.whenPressed(DpadDrive(self.robot, button=self.dpad))

        # also bound to a s d f on the 2021 keyboard
        #self.buttonA.whenPressed( AutonomousDriveTimed(self.robot, timeout=1.5) )
        self.buttonB.whenPressed(ShooterToggleFlywheel(self.robot, omega=-3450))
        #self.buttonA.whenPressed(FRCCharacterization(self.robot, timeout=60, button=self.buttonA))
        #self.buttonB.whenPressed( AutonomousRotate(self.robot, setpoint=60, timeout=4, source='dashboard') )
        #self.buttonX.whenPressed( AutonomousRotate(self.robot, setpoint=-60, timeout=4, source='dashboard', absolute=True) )
        self.buttonY.whenPressed( ShooterHood(self.robot, button=self.buttonY, power=-0.07) )
        self.buttonA.whenPressed( ShooterHood(self.robot, button=self.buttonA, power=0.07) )
        #self.buttonY.whenPressed(ShooterToggleFlywheel(self.robot, omega=-4150))
        #self.buttonA.whenPressed(ShooterToggleFlywheel(self.robot, omega=-3250))
        #self.buttonX.whenPressed(ShooterToggleFlywheel(self.robot, omega=-4800))
        # self.buttonY.whenPressed( AutonomousDrivePID(self.robot, setpoint=2, timeout=4, source='dashboard') )
        self.buttonX.whenPressed(ShooterFire(self.robot, button=self.buttonLB))

        self.buttonRB.whenPressed(ShooterFeed(self.robot, button=self.buttonRB, direction="forward"))
        #self.axisButtonRT.whenPressed(ShooterFeed(self.robot, button=self.axisButtonRT, direction="forward"))
        self.buttonLB.whenPressed(ShooterFeed(self.robot, button=self.buttonLB, direction="backward"))

        # testing vision commands
        #self.buttonY.whileHeld(DriveToBall(self.robot))
        # self.buttonX.whenPressed()

        # g h j k on the keyboard
        # self.buttonLB.whenPressed( AutonomousSlalom(self.robot)  )
        # self.buttonRB.whenPressed( AutonomousBounce(self.robot) )
        #self.buttonBack.whenPressed( AutonomousRamsete(self.robot) )
        # self.buttonStart.whenPressed(AutonomousVelocityPID(self.robot))

    def initialize_joystics(self):
        """
        Assign all buttons on the driver and co-pilot's gamepads
        Does not need to be edited once written
        :return:
        """
        self.stick = wpilib.Joystick(0)
        self.buttonA = JoystickButton(self.stick, 1)
        self.buttonB = JoystickButton(self.stick, 2)
        self.buttonX = JoystickButton(self.stick, 3)
        self.buttonY = JoystickButton(self.stick, 4)
        self.buttonLB = JoystickButton(self.stick, 5)
        self.buttonRB = JoystickButton(self.stick, 6)
        self.buttonBack = JoystickButton(self.stick, 7)
        self.buttonStart = JoystickButton(self.stick, 8)
        self.axisButtonLT = AxisButton(self.stick, 2)
        self.axisButtonRT = AxisButton(self.stick, 3)
        self.dpad = Dpad(self.stick)

        # add/change bindings if we are using more than one joystick
        if self.competition_mode:
            self.co_stick = wpilib.Joystick(1)
            self.co_buttonA = JoystickButton(self.co_stick, 1)
            self.co_buttonB = JoystickButton(self.co_stick, 2)
            self.co_buttonX = JoystickButton(self.co_stick, 3)
            self.co_buttonY = JoystickButton(self.co_stick, 4)
            self.co_buttonLB = JoystickButton(self.co_stick, 5)
            self.co_buttonRB = JoystickButton(self.co_stick, 6)
            self.co_buttonBack = JoystickButton(self.co_stick, 7)
            self.co_buttonStart = JoystickButton(self.co_stick, 8)
            self.co_povButtonUp = POVButton(self.co_stick, 0)
            self.co_povButtonDown = POVButton(self.co_stick, 180)
            self.co_povButtonRight = POVButton(self.co_stick, 90)
            self.co_povButtonLeft = POVButton(self.co_stick, 270)
            self.co_axisButtonLT = AxisButton(self.co_stick, 2)
            self.co_axisButtonRT = AxisButton(self.co_stick, 3)

    def initialize_dashboard(self):
        # dummy setpoints to speed up testing from the dashboard

        SmartDashboard.putNumber('z_distance', 2.0)
        SmartDashboard.putNumber('z_angle', 60)

        self.drive_fwd_command =  AutonomousDriveTimed(self.robot, timeout=1)
        self.rotate_command = AutonomousRotate(self.robot, setpoint=45, timeout=3, source='dashboard')
        self.autonomous_test_command = AutonomousSlalom(self.robot)
        self.autonomous_test_ramsete_command = AutonomousRamsete(self.robot)
        self.autonomous_pid_command = AutonomousDrivePID(self.robot, setpoint=2, timeout=4, source='dashboard')

        # these don't work right in 2021 as of 20210131 - keep interrupting and restarting - old ocmmand structure issue?
        #SmartDashboard.putData("Auto Ramsete", self.autonomous_test_ramsete_command)
        #SmartDashboard.putData("Auto PID", self.autonomous_pid_command)
        #SmartDashboard.putData("Auto Drive", self.drive_fwd_command )
        SmartDashboard.putData('Rotate command', self.rotate_command)

        # set up the dashboard chooser for the autonomous options
        self.obstacle_chooser = SendableChooser()
        routes = ['slalom', 'barrel', 'bounce', 'none']
        for ix, position in enumerate(routes):
            if ix == 3:
                self.obstacle_chooser.setDefaultOption(position, position)
            else:
                self.obstacle_chooser.addOption(position, position)
        wpilib.SmartDashboard.putData('obstacles', self.obstacle_chooser)

        self.path_chooser = SendableChooser()
        wpilib.SmartDashboard.putData('ramsete path', self.path_chooser)
        #choices = ['loop', 'poses', 'points', 'test', 'slalom_pw0_0.75','slalom_pw1_0.75', 'slalom_pw2_0.75', 'slalom_pw0_1.25', 'slalom_pw1_1.25',
        #           'slalom_pw2_1.25', 'slalom_pw1_1.80', 'barrel_pw1_0.75', 'barrel_pw1_1.25', 'bounce_pw1_0.75', 'bounce_pw1_1.25', 'student_pw0_0p75',
        #           'student_pw1_0p75','student_pw0_1p25', 'student_pw1_1p25']
        #choices = drive_constants.get_pathweaver_files() + ['z_loop', 'z_poses', 'z_points', 'z_test']
        choices = drive_constants.get_pathweaver_paths(simulation=self.robot.isSimulation()) + ['z_loop', 'z_poses', 'z_points', 'z_test']
        for ix, position in enumerate(choices):
            if ix == 0:
                self.path_chooser.setDefaultOption(position, position)
            else:
                self.path_chooser.addOption(position, position)

        self.velocity_chooser = SendableChooser()
        wpilib.SmartDashboard.putData('path velocity', self.velocity_chooser)
        velocities = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 6.0, 7.0]
        for ix, position in enumerate(velocities):
            if ix == 4: # 2.5 will be the default
                self.velocity_chooser.setDefaultOption(str(position), position)
            else:
                self.velocity_chooser.addOption(str(position), position)

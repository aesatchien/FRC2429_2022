import time
import wpilib
from wpilib.interfaces import GenericHID

import commands2
import commands2.button

import constants

from commands.complexauto import ComplexAuto
from commands.drivedistance import DriveDistance
from commands.defaultdrive import DefaultDrive
from commands.grabhatch import GrabHatch
from commands.halvedrivespeed import HalveDriveSpeed
from commands.releasehatch import ReleaseHatch

from subsystems.drivesubsystem import DriveSubsystem
from subsystems.hatchsubsystem import HatchSubsystem
from subsystems.elevator_subsystem import ElevatorSubsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.start_time = time.time()
        # The driver's controller
        # self.driverController = wpilib.XboxController(constants.kDriverControllerPort)
        self.driverController = wpilib.Joystick(constants.kDriverControllerPort)

        # The robot's subsystems
        self.drive = DriveSubsystem()
        self.hatch = HatchSubsystem()
        self.elevator = ElevatorSubsystem()

        # Autonomous routines

        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = DriveDistance(
            constants.kAutoDriveDistanceInches, constants.kAutoDriveSpeed, self.drive
        )

        # A complex auto routine that drives forward, drops a hatch, and then drives backward.
        self.complexAuto = ComplexAuto(self.drive, self.hatch)

        # Chooser
        self.chooser = wpilib.SendableChooser()

        # Add commands to the autonomous command chooser
        self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)
        self.chooser.addOption("Complex Auto", self.complexAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("Autonomous", self.chooser)

        self.configureButtonBindings()

        # set up default drive command
        self.drive.setDefaultCommand(
            DefaultDrive(
                self.drive,
                lambda: -self.driverController.getY(GenericHID.Hand.kLeftHand),
                lambda: 0.5*self.driverController.getTwist(),
            )
        )

    def set_start_time(self):  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_start_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return self.start_time

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        button_A = commands2.button.JoystickButton(self.driverController, 1)
        button_B = commands2.button.JoystickButton(self.driverController, 2)
        button_X = commands2.button.JoystickButton(self.driverController, 3)
        button_Y = commands2.button.JoystickButton(self.driverController, 4)
        button_RB = commands2.button.JoystickButton(self.driverController, 6)
        button_start = commands2.button.JoystickButton(self.driverController, 8)

        button_A.whenPressed(GrabHatch(self.hatch))
        button_B.whenPressed(ReleaseHatch(self.hatch))

        # set RB to half speed
        button_RB.whenHeld(HalveDriveSpeed(self, self.drive))

        # adding some elevator control commands
        button_X.whenPressed(lambda: self.elevator.raise_elevator(0.99)).whenReleased(lambda: self.elevator.raise_elevator(0))
        button_Y.whenPressed(lambda: self.elevator.start_controller(50)).whenReleased(lambda: self.elevator.stop_controller())

        # adding a compressor toggle lambda
        button_start = button_start.whenPressed(lambda: self.hatch.toggle_compressor_control())


    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()

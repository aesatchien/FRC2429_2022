# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import typing

import commands2
import commands2.button
from commands2.button import JoystickButton
import wpilib
from wpilib import SmartDashboard

from commands.arcadedrive import ArcadeDrive
from commands.autonomous_distance import AutonomousDistance
from commands.autonomous_time import AutonomousTime
from commands.turndegrees_ffwd import TurnDegreesFFWD
from commands.drivetoball import DriveToBall
from commands.findball import FindBall

from subsystems.drivetrain import Drivetrain
from subsystems.onboardio import ChannelMode, OnBoardIO
from subsystems.pixy import Pixy

import time

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        self.start_time = time.time()

        # The robot's subsystems and commands are defined here...
        self.drivetrain = Drivetrain()
        self.onboardIO = OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT)
        self.pixy = Pixy()

        # Assumes a gamepad plugged into channnel 0
        self.controller = wpilib.Joystick(0)

        # Create SmartDashboard chooser for autonomous routines
        self.chooser = wpilib.SendableChooser()

        # NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
        # that is specified when launching the wpilib-ws server on the Romi raspberry pi.
        # By default, the following are available (listed in order from inside of the board to outside):
        # - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
        # - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
        # - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
        # - PWM 2 (mapped to Arduino Pin 21)
        # - PWM 3 (mapped to Arduino Pin 22)
        #
        # Your subsystem configuration should take the overlays into account

        self._configureButtonBindings()

    def _configureButtonBindings(self):
        """Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :class:`.GenericHID` or one of its subclasses (:class`.Joystick or
        :class:`.XboxController`), and then passing it to a :class:`.JoystickButton`.
        """

        # Default command is arcade drive. This will run unless another command
        # is scheduler over it
        self.drivetrain.setDefaultCommand(self.getArcadeDriveCommand())

        # Example of how to use the onboard IO
        onboardButtonA = commands2.button.Button(self.onboardIO.getButtonAPressed)
        onboardButtonA.whenActive(
            commands2.PrintCommand("Button A Pressed")
        ).whenInactive(commands2.PrintCommand("Button A Released"))

        # Setup SmartDashboard options
        self.chooser.setDefaultOption(
            "Auto Routine Distance", AutonomousDistance(self.drivetrain)
        )
        self.chooser.addOption("Auto Routine Time", AutonomousTime(self.drivetrain))
        wpilib.SmartDashboard.putData(self.chooser)


        # CJH adding diagnostics for turning
        self.buttonA = JoystickButton(self.controller, 1)
        self.buttonB = JoystickButton(self.controller, 2)
        self.buttonX = JoystickButton(self.controller, 3)
        self.buttonY = JoystickButton(self.controller, 4)

        # setting up tests buttons for each button
        self.buttonA.whenPressed(TurnDegreesFFWD(speed=0.25, degrees=45, drive=self.drivetrain))
        self.buttonB.whenPressed(TurnDegreesFFWD(speed=0.5, degrees=90, drive=self.drivetrain))
        # self.buttonX.whenPressed(TurnDegreesFFWD(speed=0.25, degrees=-45, drive=self.drivetrain))
        # self.buttonY.whenPressed(TurnDegreesFFWD(speed=0.5, degrees=-180, drive=self.drivetrain))

<<<<<<< HEAD
        SmartDashboard.putNumber('ball_debug/kp', 0.02)
        SmartDashboard.putNumber('ball_debug/ki', 0.0)
        SmartDashboard.putNumber('ball_debug/kd', 0.0)

        self.buttonX.whileHeld(FindBall(speed=0.5, pixy=self.pixy, drive=self.drivetrain))
        self.buttonY.whileHeld(DriveToBall(pixy=self.pixy, drive=self.drivetrain))
=======
        self.buttonX.whileHeld(FindBall(pixy=self.pixy, drive=self.drivetrain, search_speed=0.5))
        self.buttonY.whileHeld(DriveToBall(pixy=self.pixy, drive=self.drivetrain, drive_speed=0.4))
>>>>>>> origin/romi_hw

    def getAutonomousCommand(self) -> typing.Optional[commands2.CommandBase]:
        return self.chooser.getSelected()

    def getArcadeDriveCommand(self) -> ArcadeDrive:
        """Use this to pass the teleop command to the main robot class.

        :returns: the command to run in teleop
        """
        dampen = 0.5
<<<<<<< HEAD

        return ArcadeDrive(
            self.drivetrain,
            lambda: dampen * -self.controller.getRawAxis(1),
            lambda: dampen * self.controller.getRawAxis(2),
=======
        return ArcadeDrive(
            self.drivetrain,
            lambda: dampen * -self.controller.getRawAxis(1),
            lambda: dampen * self.controller.getRawAxis(4),
>>>>>>> origin/romi_hw
        )

    def get_enabled_time(self):  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time
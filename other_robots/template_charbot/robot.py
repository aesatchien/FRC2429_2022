#!/usr/bin/env python3
"""
Stripped-down version of 2429's 2022 robot for frc-characterization tool
pip install frc-characterization
then run frc-characterization.exe drive new and
"""


import wpilib
from wpilib import Timer
import commands2
from robotcontainer import RobotContainer

# characterization stuff
import math
import networktables

# 2429-specific imports - need to import every subsystem you instantiate
from subsystems.drivetrain import Drivetrain  # uses SparkMax, trying to get to work with simulation


class Robot(commands2.TimedCommandRobot):
    """Main robot class"""

    # used for frc_characterization
    entries = []   # data for the frc-configuration tool
    counter = 0
    autoSpeedEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/autospeed")
    telemetryEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/telemetry")
    rotateEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/rotate")
    characterize = True
    counter = 0

    def robotInit(self):
        """Robot-wide initialization code should go here"""
        super().__init__()

        self.container = RobotContainer()
        self.drivetrain = self.container.robot_drive

        self.enabled_time = 0  # something is especially weird with the sim about this needing to be initialized in robotInit

        self.autonomousCommand = None  # initialize the placeholder command for autonomous

    def autonomousInit(self):
        """Called when autonomous mode is enabled"""
        self.enabled_time = Timer.getFPGATimestamp()
        if self.characterize:
            self.init_characterization()

    def autonomousPeriodic(self):
        if self.characterize:
            self.update_characterization()

    def teleopInit(self):
        """Called when teleop mode is enabled"""
        self.enabled_time = Timer.getFPGATimestamp()
        self.reset()
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self):
        self.drivetrain.tank_drive_volts(2, -2)
        self.drivetrain.drive.feed()
        # Scheduler.getInstance().run()

    def testPeriodic(self):
        """This function is called periodically during test mode."""
        #wpilib.LiveWindow.run()
        pass

    def disabledInit(self):
        self.reset()
        if self.characterize:
            self.clear_characterization()

    def disabledPeriodic(self):
        """This function is called periodically while disabled."""
        self.log()

    def log(self):
        # worried about too much comm during the match
        pass

    def reset(self):
        self.drivetrain.reset()

    # ---------------   FRC-CHARACTERIZATION TOOL FUNCTIONS  --------------------
    def init_characterization(self):
        self.prior_autospeed = 0
        self.data = ''
        networktables.NetworkTablesInstance.getDefault().setUpdateRate(0.010)
        self.counter = 0

    def update_characterization(self):
        now = Timer.getFPGATimestamp()
        left_position = self.drivetrain.get_position(self.drivetrain.left_encoder)
        left_rate = self.drivetrain.get_rate(self.drivetrain.left_encoder)
        right_position = self.drivetrain.get_position(self.drivetrain.right_encoder)
        right_rate = self.drivetrain.get_rate(self.drivetrain.right_encoder)
        if self.isReal():
            battery = 2*wpilib.RobotController.getVoltage6V()  # no battery voltage in python controller?
        else:
            battery = 12  #ToDo - get a real vs simulated value for this
        motor_volts = battery * math.fabs(self.prior_autospeed)
        left_motor_volts = motor_volts
        right_motor_volts = motor_volts
        autospeed = self.autoSpeedEntry.getDouble(0)
        self.prior_autospeed = autospeed

        factor = 1.0 if self.rotateEntry.getBoolean(False) else -1.0
        if self.counter  % 200 == 0:
            print(f'Autospeed:{self.autoSpeedEntry.getDouble(0)}  Rotate:{self.rotateEntry.getBoolean(False)} factor:{factor}')
        self.drivetrain.tank_drive_volts(autospeed * battery, factor * autospeed * battery)
        self.drivetrain.drive.feed()

        vals = [now, battery, autospeed, left_motor_volts, right_motor_volts, left_position, right_position, left_rate,
                right_rate, self.drivetrain.navx.getRotation2d().radians()]
        for i in vals:
            self.entries.append(i)
        self.counter += 1

    def clear_characterization(self):
        elapsed_time = Timer.getFPGATimestamp() - self.enabled_time
        self.drivetrain.tank_drive_volts(0, 0)
        self.data = str(self.entries)
        self.data = self.data[1:-2] + ', '
        self.telemetryEntry.setString(self.data)
        self.entries.clear()
        print(f'** collected {self.counter} data points in {elapsed_time} s **')
        self.data = ''

if __name__ == "__main__":
    wpilib.run(Robot)

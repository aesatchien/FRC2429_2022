#!/usr/bin/env python3
# Stripped-down version of 2020's Infinite Recharge robot for 2021's simulation and pathweaving

import wpilib
from wpilib import Timer, AddressableLED
from commandbased import CommandBasedRobot
from wpilib.command import Scheduler
from commands.autonomous_ramsete import AutonomousRamsete
from commands.autonomous_home_bounce import AutonomousBounce

# characterization stuff
import math
import networktables

# 2429-specific imports - need to import every subsystem you instantiate
from subsystems.drivetrain_sim import DriveTrainSim  # simulation only, no SparkMax devices
from subsystems.drivetrain import DriveTrain  # uses SparkMax, trying to get to work with simulation
from subsystems.shooter import Shooter
from subsystems.vision import Vision
from oi import OI

class Robot(CommandBasedRobot):
    """Main robot class"""

    # used for frc_characterization
    entries = []   # data for the frc-configuration tool
    counter = 0
    autoSpeedEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/autospeed")
    telemetryEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/telemetry")
    rotateEntry = networktables.NetworkTablesInstance.getDefault().getEntry("/robot/rotate")
    characterize = False

    def robotInit(self):
        """Robot-wide initialization code should go here"""
        super().__init__()

        if self.isReal():  # use the real drive train
            self.drivetrain = DriveTrain(self)
        else:  # use the simulated drive train
            #self.drivetrain = DriveTrainSim(self)
            self.drivetrain = DriveTrainSim(self)

        self.shooter = Shooter(self)
        self.vision = Vision()

        # oi MUST be created after all other subsystems since it uses them
        self.oi = OI(self)

        self.enabled_time = 0  # something is especially weird with the sim about this needing to be initialized in robotInit

        self.autonomousCommand = None  # initialize the placeholder command for autonomous

        led = AddressableLED(0)
        data = [AddressableLED.LEDData(255, 0, 0) for _ in range(20)]
        led.setLength(len(data))
        led.setData(data)
        led.start()



    def autonomousInit(self):
        """Called when autonomous mode is enabled"""
        self.enabled_time = Timer.getFPGATimestamp()
        if self.characterize:
            self.init_characterization()
        else:
            # self.autonomousCommand = FRCCharacterization(self, button=self.oi.buttonA, timeout=60)
            if self.isReal():
                self.autonomousCommand = AutonomousBounce(self)  # bounce paths - four of them
            else:
                self.autonomousCommand = AutonomousRamsete(self)  # single paths from the drop down

            self.autonomousCommand.start()

    def autonomousPeriodic(self):
        if self.characterize:
            self.update_characterization()
        else:
            Scheduler.getInstance().run()

    def teleopInit(self):
        """Called when teleop mode is enabled"""
        self.enabled_time = Timer.getFPGATimestamp()
        self.reset()
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()

    def teleopPeriodic(self):
        Scheduler.getInstance().run()

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
        left_position = self.drivetrain.get_position(self.drivetrain.l_encoder)
        left_rate = self.drivetrain.get_rate(self.drivetrain.l_encoder)
        right_position = self.drivetrain.get_position(self.drivetrain.r_encoder)
        right_rate = self.drivetrain.get_rate(self.drivetrain.r_encoder)
        if self.isReal():
            battery = 2*wpilib.RobotController.getVoltage6V()  # no battery voltage in python controller?
        else:
            battery = 11  #ToDo - get a real vs simulated value for this
        motor_volts = battery * math.fabs(self.prior_autospeed)
        left_motor_volts = motor_volts
        right_motor_volts = motor_volts
        autospeed = self.autoSpeedEntry.getDouble(0)
        self.prior_autospeed = autospeed

        factor = -1.0 if self.rotateEntry.getBoolean(False) else 1.0
        self.drivetrain.drive.tankDrive(factor * autospeed, autospeed, False)
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

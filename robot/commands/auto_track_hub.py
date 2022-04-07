from math import copysign
import commands2
from wpilib import SmartDashboard
from wpimath.controller import PIDController

import constants

SmartDashboard.putNumber('AutoTrackHub/kp', .01)
SmartDashboard.putNumber('AutoTrackHub/ki', 0)
SmartDashboard.putNumber('AutoTrackHub/kd', 0.0001)
SmartDashboard.putNumber('AutoTrackHub/ff', 0.15)

class AutoTrackHub(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive, shooter, vision, pneumatics) -> None:
        super().__init__()
        self.setName('AutoTrackHub')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.shooter = shooter
        self.vision = vision
        self.pneumatics = pneumatics

        self.controller = PIDController(0.005, 0.005, 0.0001)
        self.controller.setIntegratorRange(0, 0.12)  # integral reacts quickly but is capped at a low value
        self.feed_forward = 0.15
        self.min_approach = 0.7

        self.counter = 0
        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.drive.arcade_drive(0, 0)
        self.counter = 0

        self.controller.setP(SmartDashboard.getNumber('AutoTrackHub/kp', 0.014))
        self.controller.setI(SmartDashboard.getNumber('AutoTrackHub/ki', 0.014))
        self.controller.setD(SmartDashboard.getNumber('AutoTrackHub/kd', 0.0000))
        self.feed_forward = SmartDashboard.getNumber('AutoTrackHub/ff', 0.15)

        self.shooter.set_flywheel(2500)

        print(f"\nkp: {self.controller.getP()}\tki: {self.controller.getI()}\tkd: {self.controller.getD()}\tff: {self.feed_forward}")

        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        self.drive.feed()
        self.counter += 1

        if self.counter % 10 == 0:
            (hub_detected, rotation_offset, distance) = self.vision.getHubValues()

            error = abs(rotation_offset)

            if hub_detected and (error > 2):
                # print(f"attempting to turn  to hub at {rotation_offset:2.1f}...")
                if constants.k_is_simulation:  # keep sim from going crazy
                    orientation = self.drive.navx.getAngle()
                    SmartDashboard.putNumber('/sim/track_sp', orientation + rotation_offset)
                    twist_output = self.controller.calculate(orientation,  orientation + rotation_offset)
                    twist_output = min(0.2, twist_output) if twist_output > 0 else max(-.2, twist_output)
                    twist_output += copysign(1, rotation_offset) * self.feed_forward * 0.2
                else:
                    orientation = self.drive.navx.getAngle()
                    twist_output = self.controller.calculate(orientation, orientation + rotation_offset)
                    twist_output += copysign(1, rotation_offset) * self.feed_forward

                #thrust_output = 0.35 if distance > self.min_approach else 0
                self.drive.arcade_drive(0, twist_output)

                # update shooter RPM 10 times per second
                if distance < 3:
                    if self.pneumatics.shooter_hood_extended:
                        self.pneumatics.set_shooter_hood_position(position='retract')

                    rpm = self.vision.getShooterRpmNoHood()
                elif distance >= 3 and distance <= 3.2:
                    if self.pneumatics.shooter_hood_extended:
                        rpm = self.vision.getShooterHoodRpm()
                    else:
                        rpm = self.vision.getShooterRpmNoHood()
                elif distance > 3.2:
                    if not self.pneumatics.shooter_hood_extended:
                        self.pneumatics.set_shooter_hood_position(position='extend')

                    rpm = self.vision.getShooterHoodRpm()

                self.shooter.set_flywheel(rpm)

            else:
                # print('hub not detected')
                self.drive.arcade_drive(0, 0)



    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
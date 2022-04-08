from math import copysign
import commands2
from wpilib import SmartDashboard
from wpimath.controller import PIDController

import constants

class AutoTrackHub(commands2.CommandBase):  # change the name for your command

    kp = 0.006
    ki = 0.005
    kd = 0
    feed_forward = 0.15

    SmartDashboard.putNumber('AutoTrackHub/kp', kp)
    SmartDashboard.putNumber('AutoTrackHub/ki', ki)
    SmartDashboard.putNumber('AutoTrackHub/kd', kd)
    SmartDashboard.putNumber('AutoTrackHub/ff', feed_forward)

    def __init__(self, container, drive, shooter, vision, pneumatics, autonomous=False) -> None:
        super().__init__()
        self.setName('AutoTrackHub')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.shooter = shooter
        self.vision = vision
        self.pneumatics = pneumatics

        self.controller = PIDController(Kp=self.kp, Ki=self.ki, Kd=self.kd)
        self.controller.setIntegratorRange(0, 0.1)  # integral reacts quickly but is capped at a low value
        self.min_approach = 0.7

        self.autonomous = autonomous
        self.error = 2.1

        self.counter = 0
        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.drive.arcade_drive(0, 0)
        self.counter = 0

        self.controller.reset()
        self.controller.setP(SmartDashboard.getNumber('AutoTrackHub/kp', self.kp))
        self.controller.setI(SmartDashboard.getNumber('AutoTrackHub/ki', self.ki))
        self.controller.setD(SmartDashboard.getNumber('AutoTrackHub/kd', self.kd))
        self.feed_forward = SmartDashboard.getNumber('AutoTrackHub/ff', self.feed_forward)

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

            self.error = abs(rotation_offset)

            if hub_detected:
                # print(f"attempting to turn  to hub at {rotation_offset:2.1f}...")
                twist_output = 0
                if self.error > 2:
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
        if self.autonomous:
            return self.error < 2
        else:
            return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
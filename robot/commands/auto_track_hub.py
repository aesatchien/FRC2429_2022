from math import copysign
import commands2
from wpilib import SmartDashboard
from wpimath.controller import PIDController

import constants


class AutoTrackHub(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive, shooter, vision) -> None:
        super().__init__()
        self.setName('AutoTrackHub')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.shooter = shooter
        self.vision = vision

        self.controller = PIDController(0.01, 0, 0.0001)
        self.feed_forward = 0.15
        self.min_approach = 0.7

        self.counter = 0
        self.addRequirements(drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        self.drive.arcade_drive(0, 0)
        self.counter = 0

        # self.container.robot_shooter.set_flywheel(rpm=2500)

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

            else:
                # print('hub not detected')
                self.drive.arcade_drive(0, 0)

        self.shooter.set_flywheel(self.vision.getShooterRpm())

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
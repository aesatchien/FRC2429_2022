#!/usr/bin/env python3

import typing
import wpilib
import commands2
from commands2 import TimedCommandRobot
from robotcontainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # self.container.robot_drive.set_brake_mode(mode='coast')

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""

        self.container.set_start_time()  # putting this after the scheduler is bad

        self.autonomousCommand = self.container.get_autonomous_command()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

        # self.container.robot_drive.set_brake_mode(mode='brake')

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.

        self.container.set_start_time()  # putting this after the scheduler is bad
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # self.container.robot_drive.set_brake_mode(mode='brake')


    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()

        # self.container.robot_drive.set_brake_mode(mode='brake')


if __name__ == "__main__":
    debug = False

# CJH trying to figure out issues with load times and watchdog overruns - advanced students can google profiling
    if debug:
        import cProfile, pstats
        cProfile.run('wpilib.run(MyRobot)', 'stats')
        print('Starting with profiling')
        profiler = cProfile.Profile()
        profiler.enable()
        # cProfile.run('wpilib.run(MyRobot)')
        wpilib.run(MyRobot)
        profiler.disable()
        stats = pstats.Stats(profiler).sort_stats('ncalls')
        stats.print_stats()
        print('Finished with profiling')

    else:
        print('Skipping stats')
        wpilib.run(MyRobot)

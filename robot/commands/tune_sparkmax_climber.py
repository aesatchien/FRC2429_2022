import time

import commands2
from wpilib import SmartDashboard
from networktables import NetworkTablesInstance
import rev
from subsystems.climber import Climber
import constants

class TuneSparkmaxClimber(commands2.CommandBase):  # change the name for your command

    ntinst = NetworkTablesInstance.getDefault()
    sparkmax_table = ntinst.getTable('Rev_Climber')
    keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
    PID_multiplier = 1000  # small numbers do not show up well on the dash
    for key in keys:
        sparkmax_table.putNumber(key + '_pos', PID_multiplier * constants.PID_dict_pos[key])
        sparkmax_table.putNumber(key + '_vel', PID_multiplier * constants.PID_dict_vel[key])
    sparkmax_table.putNumber('vel_sp', 1)
    sparkmax_table.putNumber('pos_sp', 1)
    sparkmax_table.putNumber('pos_arbff', 0)
    sparkmax_table.putNumber('vel_arbff', 0)
    sparkmax_table.putNumber('vel_max_accel', constants.smartmotion_maxacc)
    sparkmax_table.putNumber('pos_max_accel', constants.smartmotion_maxacc)

    def __init__(self, container, climber:Climber, setpoint=1, control_type='position', spin=False) -> None:
        super().__init__()
        self.setName('TuneSparkmaxClimber')  # change this to something appropriate for this command
        self.container = container
        self.climber = climber
        self.addRequirements(self.drive)  # commandsv2 version of requirements

        self.tolerance = 0.1  # 10 cm
        self.setpoint = setpoint
        self.control_type = control_type
        self.spin = spin
        self.max_vel = 3

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        # update the PIDs in the controllers
        for key in self.keys:
            self.climber.PID_dict_pos[key] = self.sparkmax_table.getNumber(key + '_pos', 0) / self.PID_multiplier
            self.climber.PID_dict_vel[key] = self.sparkmax_table.getNumber(key + '_vel', 0) / self.PID_multiplier
        self.climber.configure_controllers(pid_only=False)

        # get setpoint from the dash
        if self.control_type == 'position':
            self.setpoint = self.sparkmax_table.getNumber('pos_sp', 1)
            self.max_accel = self.sparkmax_table.getNumber('pos_max_accel', 500)
            self.k_arb_ff = self.sparkmax_table.getNumber('pos_arbff', 0)

        elif self.control_type == 'velocity':
            self.setpoint = self.sparkmax_table.getNumber('vel_sp', 1)
            self.max_accel = self.sparkmax_table.getNumber('vel_max_accel', 500)
            self.k_arb_ff = self.sparkmax_table.getNumber('vel_arbff', 0)

        # multipliers = [1.0, 1.0, 1.0, 1.0] if self.spin else [1.0, 1.0, -1.0, -1.0]
        time.sleep(0.1)

        # set the control type
        if self.control_type == 'position':
            position = self.climber.climber_left_encoder.getPosition()
            self.initial_position = position

            self.climber.climber_left_controller.setSmartMotionMaxAccel(self.max_accel, 0)
            self.climber.climber_right_controller.setSmartMotionMaxAccel(self.max_accel, 0)

            # [pid_controller.setSmartMotionMaxVelocity(self.max_vel, 0) for pid_controller in self.drive.pid_controllers]

            self.climber_climber_left_controller.setReference(self.setpoint + position,
                                        rev.CANSparkMaxLowLevel.ControlType.kSmartMotion, 0, arbFeedforward=self.k_arb_ff)
            self.climber_climber_right_controller.setReference(self.setpoint + position,
                                        rev.CANSparkMaxLowLevel.ControlType.kSmartMotion, 0, arbFeedforward=self.k_arb_ff)
        elif self.control_type == 'velocity':  # velocity needs to be continuous

            self.climber.climber_left_controller.setSmartMotionMaxAccel(self.max_accel, 1)
            self.climber.climber_right_controller.setSmartMotionMaxAccel(self.max_accel, 1)

            self.climber_climber_left_controller.setReference(self.setpoint, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
            self.climber_climber_right_controller.setReference(self.setpoint, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
        else:
            pass

    def execute(self) -> None:
        self.container.robot_drive.feed()


    def isFinished(self) -> bool:
        if self.control_type == 'velocity':
            return False  # run until cancelled
        elif self.control_type == 'position':
            # return False
            error = self.setpoint - (self.climber.climber_left_encoder.getPosition() - self.initial_position)
            return abs(error) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:

        self.climber_climber_left_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
        self.climber_climber_right_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)

        for i in range (10):
            time.sleep(0.02)
            self.container.robot_drive.feed()
        # self.drive.tank_drive_volts(0, 0)  # stop the robot

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



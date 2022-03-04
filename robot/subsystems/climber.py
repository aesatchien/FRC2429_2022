from commands2 import SubsystemBase
from wpilib import SmartDashboard
from wpimath.filter import MedianFilter
import rev

from networktables import NetworkTablesInstance, NetworkTables

import constants


class Climber(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Climber')
        self.counter = 0

        self.current_filter = MedianFilter(5)

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.climber_left_neo = rev.CANSparkMax(constants.k_climber_left_port, motor_type)
        self.climber_right_neo = rev.CANSparkMax(constants.k_climber_right_port, motor_type)

        self.climber_left_encoder = self.climber_left_neo.getEncoder()
        self.climber_right_encoder = self.climber_right_neo.getEncoder()

        self.climber_left_controller = self.climber_left_neo.getPIDController()
        self.climber_right_controller = self.climber_right_neo.getPIDController()

        self.climber_right_neo.follow(self.climber_left_neo, invert=True)

        # add encoder later
        ntinst = NetworkTablesInstance.getDefault()
        self.climber_table = ntinst.getTable('Climber')
        self.climber_voltage = self.climber_table.getEntry('voltage')
        self.climber_current = self.climber_table.getEntry('current')

    def set_voltage(self, voltage):
        self.climber_left_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage)

    def stop_motor(self):
        self.climber_left_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)


    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 2 == 0:
            self.current_filter.calculate(self.climber_left_neo.getOutputCurrent())  # update the current filter

        if self.counter % 10 == 0:

            # ten per second updates
            SmartDashboard.putNumber('/climber/climber voltage', self.climber_left_neo.getAppliedOutput())
            SmartDashboard.putNumber('/climber/climber current', self.current_filter.calculate(self.climber_left_neo.getOutputCurrent()))

            self.climber_voltage.setValue(self.climber_left_neo.getAppliedOutput())  # does not like setNumber for some reason
            self.climber_current.setValue(self.current_filter.calculate(self.climber_left_neo.getOutputCurrent()))




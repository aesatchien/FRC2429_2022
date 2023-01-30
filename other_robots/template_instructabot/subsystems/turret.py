import time
from commands2 import SubsystemBase #commands2 --> scheduler that tells the robot what to do. Helps with avoiding conflicting commands.
from wpilib import SmartDashboard
from wpimath.filter import MedianFilter
import rev

import constants


class Turret(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Turret')
        self.counter = 0


        #telling the motor what it is.
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.turret_left_neo = rev.CANSparkMax(6, motor_type) #looking for a motor with id6

        self.turret_left_encoder = self.turret_left_neo.getEncoder()
        # self.turret_left_neo.getA

        conversion_factor = 1 
        self.turret_left_encoder.setPositionConversionFactor(conversion_factor)  
        self.turret_left_encoder.setVelocityConversionFactor(conversion_factor / 60) #rev per second
        self.start_angle = 0  # we start at 54 degrees from the vertical
        self.turret_left_encoder.setPosition(self.start_angle)  # may need to have to worry about the sign here

        self.turret_enable = False
        SmartDashboard.putBoolean('turret_state', self.turret_enable)

        self.turret_left_controller = self.turret_left_neo.getPIDController()

    def set_voltage(self, voltage):
        self.turret_left_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage) #setReference tells the motor to do something (value, enum_type). 
        self.turret_enable = True
        SmartDashboard.putBoolean('turret_state', self.turret_enable)

    def stop_motor(self):
        self.turret_left_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.turret_enable = False
        SmartDashboard.putBoolean('turret_state', self.turret_enable)

    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 10 == 0:
            # five per second updates. putNumber() puts the number on the gui
            SmartDashboard.putNumber('turret_voltage', self.turret_left_neo.getAppliedOutput() * 12)
            SmartDashboard.putNumber('turret_position', self.turret_left_encoder.getPosition())
            SmartDashboard.putNumber('turret_current', self.turret_left_neo.getOutputCurrent())
            SmartDashboard.putNumber('turret_velocity', self.turret_left_encoder.getVelocity())


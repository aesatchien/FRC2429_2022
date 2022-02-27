from commands2 import SubsystemBase
from wpilib import SmartDashboard

import rev

import constants


class Climber(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Climber')
        self.counter = 0

        self.climber_left_neo = rev.CANSparkMax(constants.k_climber_left_port, rev.MotorType.kBrushless)
        self.climber_right_neo = rev.CANSparkMax(constants.k_climber_right_port, rev.MotorType.kBrushless)

        self.climber_left_encoder = self.climber_left_neo.getEncoder()
        self.climber_right_encoder = self.climber_right_neo.getEncoder()

        self.climber_left_controller = self.climber_left_neo.getPIDController()
        self.climber_right_controller = self.climber_right_neo.getPIDController()

        self.climber_right_neo.follow(self.climber_left_neo, invert=True)

        # add encoder later


    def set_voltage(self, voltage):
        self.climber_left_controller.setReference(voltage, rev.ControlType.kVoltage)
    
    def stop_motor(self):
        self.climber_left_controller.setReference(0, rev.ControlType.kVoltage)


    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 5 == 0:

            # ten per second updates
            
            SmartDashboard.putNumber('climber voltage', self.climber_left_neo.getBusVoltage())
            






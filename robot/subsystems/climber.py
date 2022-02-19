from commands2 import SubsystemBase
from wpilib import SmartDashboard, Spark

import rev

import constants


class Climber(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Climber')

        self.left_motor = Spark(constants.k_climber_left_port)
        self.right_motor = Spark(constants.k_climber_right_port)
        self.right_motor.setInverted(True)
        # add encoder later


    def set_velocity(self, velocity):
        self.left_motor.set(velocity)
        self.right_motor.set(velocity)
    
    def stop_motor(self):
        self.left_motor.stopMotor()
        self.right_motor.stopMotor()
 






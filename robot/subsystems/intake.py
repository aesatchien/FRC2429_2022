from commands2 import SubsystemBase
from wpilib import Spark

import rev

import constants


class Intake(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Intake')

        self.intake_775 = Spark(constants.k_intake_motor_port)
        

    def set_velocity(self, velocity):
        self.intake_775.set(velocity)
    
    def stop_motor(self):
        self.intake_775.stopMotor()
    
    def get_velocity(self):
        return self.intake_775.get()
 

    
        


        

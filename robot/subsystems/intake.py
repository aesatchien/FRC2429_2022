from commands2 import SubsystemBase
from wpilib import Spark, SmartDashboard

import constants


class Intake(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Intake')

        self.intake_775 = Spark(constants.k_intake_motor_port)
        
        self.intake_775.setInverted(True)

        self.intake_enable = False
        self.counter = 0
        
    def set_velocity(self, velocity):
        self.intake_775.set(velocity)
        self.intake_enable = True

    def stop_motor(self):
        self.intake_775.stopMotor()
        self.intake_enable = False

    def get_velocity(self):
        return self.intake_775.get()

    def toggle_intake_motor(self, velocity):
        if self.intake_enable:
            self.stop_motor()
        else:
            self.set_velocity(velocity)


    def periodic(self) -> None:
        
        self.counter += 1

        if self.counter % 5 == 0:

            # ten per second updates
            
            SmartDashboard.putBoolean('/intake/intake motor state', self.intake_enable)
 

    
        


        

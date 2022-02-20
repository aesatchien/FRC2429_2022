# drivetrain to use both in sim and robot mode - sim handles the Sparkmax now
# started 2022 0102 to update to commands2

from commands2 import SubsystemBase
from wpilib import SmartDashboard
import rev

import constants


class Shooter(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Shooter')
        self.counter = 0

        # initialize motors
        # looking from back to front
        motor_type = rev.MotorType.kBrushless
        self.flywheel_left = rev.CANSparkMax(constants.k_flywheel_left_neo_port, motor_type)
        self.flywheel_right = rev.CANSparkMax(constants.k_flywheel_right_neo_port, motor_type)
        self.flywheel_first_stage = rev.CANSparkMax(constants.k_flywheel_stage_one_neo_port, motor_type)

        self.flywheel_right.follow(self.flywheel_left, invert=True)
        #invert on of the first tage motor

        # encoders
        self.flywheel_left_encoder = self.flywheel_left.getEncoder()
        self.flywheel_first_stage_encoder = self.flywheel_first_stage.getEncoder()

        # controller
        self.flywheel_left_controller = self.flywheel_left.getPIDController()
        self.flywheel_first_stage_controller = self.flywheel_first_stage.getPIDController()


    def set_flywheel(self, rpm):
        self.flywheel_left_controller.setReference(rpm, rev.ControlType.kSmartVelocity, 0)

    def set_first_stage(self, rpm):
        self.flywheel_first_stage_controller.setReference(rpm, rev.ControlType.kSmartVelocity, 0)
    
    def stop_shooter(self):
        self.flywheel_left_controller.setReference(0, rev.ControlType.kVoltage)
        #self.flywheel_first_stage_controller.setReference(0, rev.ControlType.kVoltage)

    def get_flywheel(self):
        return self.flywheel_left_encoder.getVelocity()
        

    """def periodic(self) -> None:
        
        self.counter += 1

        if self.counter % 5 == 0:

            # ten per second updates
            
            SmartDashboard.putNumber('shooter rpm', self.flywheel_left_encoder.getVelocity())"""

            

            
    






    






        
        
        

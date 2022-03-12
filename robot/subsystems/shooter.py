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
        self.PID_dict_vel = {'kP': 0.00021, 'kI': 0, 'kD': 0, 'kIz': 0, 'kFF': 0.000192}
        self.smartmotion_maxvel = 2501  # rpm
        self.smartmotion_maxacc = 5001
        self.current_limit = 35

        # initialize motors
        # looking from back to front
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.flywheel_left = rev.CANSparkMax(constants.k_flywheel_left_neo_port, motor_type)
        self.flywheel_right = rev.CANSparkMax(constants.k_flywheel_right_neo_port, motor_type)
        #self.flywheel_first_stage = rev.CANSparkMax(constants.k_flywheel_stage_one_neo_port, motor_type)

        self.flywheel_left.setInverted(False) # inverted left so positive rpm is shooting
        self.flywheel_right.follow(self.flywheel_left, invert=True)
        #the follower is inverted

        # encoders
        self.flywheel_left_encoder = self.flywheel_left.getEncoder()
        #self.flywheel_first_stage_encoder = self.flywheel_first_stage.getEncoder()

        # controller
        self.flywheel_left_controller = self.flywheel_left.getPIDController()
        #self.flywheel_first_stage_controller = self.flywheel_first_stage.getPIDController()
        self.flywheel_left_controller.setP(0)

        # toggle state
        self.shooter_enable = False

        self.set_pids()

    def set_flywheel(self, rpm):
        self.flywheel_left_controller.setReference(rpm, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 0)
        self.shooter_enable = True

    #def set_first_stage(self, rpm):
        #self.flywheel_first_stage_controller.setReference(rpm, rev.ControlType.kVoltage, 0)
    
    def stop_shooter(self):
        self.flywheel_left_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        #self.flywheel_first_stage_controller.setReference(0, rev.ControlType.kVoltage)
        self.shooter_enable = False

    def get_flywheel(self):
        return self.flywheel_left_encoder.getVelocity()
    
    def toggle_shooter(self, rpm):
        if self.shooter_enable:
            self.stop_shooter()
        else:
            self.set_flywheel(rpm)

    def set_pids(self, burn_flash=False):
        self.error_dict = {}
        i = 0
        self.error_dict.update({'kP0_' + str(i): self.flywheel_left_controller.setP(self.PID_dict_vel['kP'], 0)})
        self.error_dict.update({'kI0_' + str(i): self.flywheel_left_controller.setI(self.PID_dict_vel['kI'], 0)})
        self.error_dict.update({'kIz0_' + str(i): self.flywheel_left_controller.setIZone(self.PID_dict_vel['kIz'], 0)})
        self.error_dict.update({'kD0_' + str(i): self.flywheel_left_controller.setD(self.PID_dict_vel['kD'], 0)})
        self.error_dict.update({'kD0_' + str(i): self.flywheel_left_controller.setFF(self.PID_dict_vel['kFF'], 0)})
        self.error_dict.update({'Accel0_' + str(i): self.flywheel_left_controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 0)})  #
        self.error_dict.update({'Vel0_' + str(i): self.flywheel_left_controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 0)})

        # print(self.error_dict)
        if burn_flash:
            self.flywheel_left.burnFlash()

    def periodic(self) -> None:
        
        self.counter += 1

        if self.counter % 25 == 0:
            # not too often
            SmartDashboard.putNumber('shooter_rpm', self.flywheel_left_encoder.getVelocity())
            SmartDashboard.putBoolean('shooter_state', self.shooter_enable)
            SmartDashboard.putBoolean('shooter_ready', self.flywheel_left_encoder.getVelocity() > 1800)
            
            

            

            
    






    






        
        
        

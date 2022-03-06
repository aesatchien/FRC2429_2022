from commands2 import SubsystemBase
from wpilib import SmartDashboard, AnalogInput

import rev

import constants


class Indexer(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Indexer')
        self.counter = 0

        self.ball_sensor = AnalogInput(0)

        # motor
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.indexer_neo = rev.CANSparkMax(constants.k_indexer_neo_port, motor_type)
        self.flywheel_first_stage = rev.CANSparkMax(constants.k_flywheel_stage_one_neo_port, motor_type)

        # encoder
        self.indxer_encoder = self.indexer_neo.getEncoder()
        self.flywheel_first_stage_encoder = self.flywheel_first_stage.getEncoder()
        
        # controller
        self.indexer_controller = self.indexer_neo.getPIDController()
        self.flywheel_first_stage_controller = self.flywheel_first_stage.getPIDController()

        # invert
        self.indexer_neo.setInverted(False)

        # indexer state
        self.indexer_enable = False


    def set_voltage(self, voltage):
        self.indexer_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.flywheel_first_stage_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.indexer_enable = True
    
    def stop_motor(self):
        self.indexer_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.flywheel_first_stage_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)  
        self.indexer_enable = False

    def toggle_indexer(self, voltage):
        if self.indexer_enable:
            self.stop_motor()
            self.indexer_enable = False
        else:
            self.set_voltage(voltage)
            self.indexer_enable = True


    def periodic(self) -> None:
        
        if self.counter % 25 == 0:
            SmartDashboard.putBoolean('indexer state', self.indexer_enable)
            SmartDashboard.putNumber('indexer ball sensor', self.ball_sensor.getValue())


            
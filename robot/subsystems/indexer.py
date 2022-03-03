from commands2 import SubsystemBase
from wpilib import SmartDashboard

import rev

import constants


class Indexer(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Indexer')

        self.indexer_neo = rev.CANSparkMax(constants.k_indexer_neo_port, rev.MotorType.kBrushless)
        self.flywheel_first_stage = rev.CANSparkMax(constants.k_flywheel_stage_one_neo_port, rev.MotorType.kBrushless)


        self.indxer_encoder = self.indexer_neo.getEncoder()
        self.flywheel_first_stage_encoder = self.flywheel_first_stage.getEncoder()

        self.indexer_controller = self.indexer_neo.getPIDController()
        self.flywheel_first_stage_controller = self.flywheel_first_stage.getPIDController()


        self.indexer_neo.setInverted(False)


    def set_voltage(self, voltage):
        self.indexer_controller.setReference(voltage, rev.ControlType.kVoltage)
        self.flywheel_first_stage_controller.setReference(voltage, rev.ControlType.kVoltage)
    
    def stop_motor(self):
        self.indexer_controller.setReference(0, rev.ControlType.kVoltage)
        self.flywheel_first_stage_controller.setReference(0, rev.ControlType.kVoltage)        

        
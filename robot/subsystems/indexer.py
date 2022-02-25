from commands2 import SubsystemBase
from wpilib import SmartDashboard

import rev

import constants


class Indexer(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Indexer')

        self.indexer_neo = rev.CANSparkMax(constants.k_indexer_neo_port, rev.MotorType.kBrushless)

        self.indxer_encoder = self.indexer_neo.getEncoder()

        self.indexer_controller = self.indexer_neo.getPIDController()

        self.indexer_neo.setInverted(True)


    def set_velocity(self, velocity):
        self.indexer_controller.setReference(velocity, rev.ControlType.kVoltage, 0)
    
    def stop_motor(self):
        self.indexer_controller.setReference(0, rev.ControlType.kVoltage)

        
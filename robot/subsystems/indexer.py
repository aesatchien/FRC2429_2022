from commands2 import SubsystemBase
from wpilib import SmartDashboard

import rev
import constants


class Indexer(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Indexer')
        self.counter = 0

        # motor
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.indexer_sparkmax = rev.CANSparkMax(constants.k_flywheel_stage_one_neo_port, motor_type)

        # encoder
        self.indexer_encoder = self.indexer_sparkmax.getEncoder()
        
        # controller
        self.indexer_controller = self.indexer_sparkmax.getPIDController()

        # indexer state
        self.indexer_enabled = False
        SmartDashboard.putBoolean('indexer_state', self.indexer_enabled)

    def set_voltage(self, voltage):
        self.indexer_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.indexer_enabled = True
        SmartDashboard.putBoolean('indexer_state', self.indexer_enabled)

    def stop_motor(self):
        self.indexer_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.indexer_enabled = False
        SmartDashboard.putBoolean('indexer_state', self.indexer_enabled)
        
    def toggle_indexer(self, voltage):
        if self.indexer_enabled:
            self.stop_motor()
        else:
            self.set_voltage(voltage)

    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 25 == 0:
            # moved the on/off state to the functions themselves
            SmartDashboard.putNumber('indexer_voltage', self.indexer_sparkmax.getAppliedOutput())

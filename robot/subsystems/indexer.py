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
        self.indexer_stage_one = rev.CANSparkMax(constants.k_indexer_neo_port, motor_type)
        self.indexer_stage_two = rev.CANSparkMax(constants.k_flywheel_stage_one_neo_port, motor_type)

        # encoder
        self.indexer_stage_one_encoder = self.indexer_stage_one.getEncoder()
        self.indexer_stage_two_encoder = self.indexer_stage_two.getEncoder()
        
        # controller
        self.indexer_stage_one_controller = self.indexer_stage_one.getPIDController()
        self.indexer_stage_two_controller = self.indexer_stage_two.getPIDController()

        # invert
        self.indexer_stage_one.setInverted(False)

        # indexer state
        self.indexer_stage_one_enable = False
        self.indexer_stage_two_enable = False
        self.indexer_enabled = False
        SmartDashboard.putBoolean('indexer_state', self.indexer_enabled)


        #self.stages = []
        # ports = [constants.k_indexer_neo_port, constants.k_flywheel_stage_one_neo_port]
        # for port in ports:
        #     spark_max = rev.CANSparkMax(port, motor_type)
            
        #     self.stages.append({
        #         'controller': spark_max.getPIDController(),
        #         'enabled': False
        #     })

    # def set_stage_voltage(self, stage_slot: int, voltage: float) -> None:
    #     stage = self.stages[stage_slot]
    #     stage['controller'].setReference(voltage,rev.CANSparkMaxLowLevel.ControlType.kVoltage)
    #     stage['enabled'] = True

    # def stop_stage(self, stage_slot: int) -> None:
    #     stage = self.stages[stage_slot]
    #     stage['controller'].setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
    #     stage['enabled'] = False

    


    def set_voltage_stage_one(self, voltage):
        self.indexer_stage_one_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.indexer_stage_one_enable = True


    def set_voltage_stage_two(self, voltage):
        self.indexer_stage_two_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.indexer_stage_two_enable = True


    def stop_stage_one_motor(self):
        self.indexer_stage_one_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.indexer_stage_one_enable = False

    def stop_stage_two_motor(self):
        self.indexer_stage_two_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.indexer_stage_two_enable = False

    def set_voltage(self, voltage):
        self.set_voltage_stage_one(voltage)
        self.set_voltage_stage_two(voltage)
        self.indexer_enabled = True
        SmartDashboard.putBoolean('indexer_state', self.indexer_enabled)

    def stop_motor(self):
        self.stop_stage_one_motor()
        self.stop_stage_two_motor()
        self.indexer_enabled = False
        SmartDashboard.putBoolean('indexer_state', self.indexer_enabled)


    def toggle_stage_one(self, voltage):
        if self.indexer_stage_one_enable:
            self.stop__stage_one_motor()
        else:
            self.set_voltage_stage_one(voltage)
            
    def toggle_stage_two(self, voltage):
        if self.indexer_stage_two_enable:
            self.stop__stage_two_motor()
        else:
            self.set_voltage_stage_one(voltage)
        
    def toggle_indexer(self, voltage):
        if self.indexer_enabled:
            self.stop_motor()
        else:
            self.set_voltage(voltage)

    def check_enabled(self):
        if self.indexer_stage_two_enable and self.indexer_stage_one_enable:
            self.indexer_enabled = True
        else:
            self.indexer_enabled = False

    def periodic(self) -> None:

        self.counter += 1

        if self.counter % 25 == 0:
            # moved the on/off state to the functions themselves
            SmartDashboard.putNumber('indexer_v1', self.indexer_stage_one.getAppliedOutput())
            SmartDashboard.putNumber('indexer_v2', self.indexer_stage_two.getAppliedOutput())
            # SmartDashboard.putNumber('indexer_ballsensor', self.ball_sensor.getValue())
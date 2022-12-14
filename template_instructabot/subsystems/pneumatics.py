import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, Solenoid, Compressor, AnalogInput, DoubleSolenoid

import constants


class Pneumatics(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Pneumatics')
        self.counter = 0

        self.compressor = Compressor(0, wpilib.PneumaticsModuleType.CTREPCM)
        self.small_piston = DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 4, 5)
        self.large_piston = DoubleSolenoid(wpilib.PneumaticsModuleType.CTREPCM, 0, 1)

        self.pressure_sensor = AnalogInput(0)

        self.close_loop_enable = True

        competition = False
        if competition:
            self.start_compressor()
        else:
            self.stop_compressor()

        # Decide on init piston position
        self.small_piston_extended = False
        self.large_piston_extended = False
        self.set_small_piston_position(position='retract')  # make sure this is set since it remembers previous
        self.set_large_piston_position(position='retract')


    def stop_compressor(self):
        # self.compressor.stop()  # deprecated in 2022
        self.compressor.disable()   # should now be disable, as of 2022
        self.close_loop_enable = False
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    def start_compressor(self):
        # self.compressor.setClosedLoopControl(True)
        self.compressor.enableDigital()  # setClosedLoopControl(True) is no longer a call in 2022
        self.close_loop_enable = True
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)

    def toggle_compressor(self):
        if self.close_loop_enable:
            self.stop_compressor()
        else:
            self.start_compressor()

    def set_small_piston_position(self, position):
        if position == 'extend':
            self.small_piston_extended = True
            self.small_piston.set(DoubleSolenoid.Value.kReverse)
        elif position == 'retract':
            self.small_piston_extended = False
            self.small_piston.set(DoubleSolenoid.Value.kForward)
        SmartDashboard.putBoolean('small_piston_extended', self.small_piston_extended)

    def toggle_small_piston_position(self):
        if self.small_piston_extended:
            self.set_small_piston_position('retract')
        else:
            self.set_small_piston_position('extend')

    def set_large_piston_position(self, position):
        if position == 'extend':
            self.large_piston_extended = True
            self.large_piston.set(DoubleSolenoid.Value.kReverse)
        elif position == 'retract':
            self.large_piston_extended = False
            self.large_piston.set(DoubleSolenoid.Value.kForward)
        SmartDashboard.putBoolean('large_piston_extended', self.large_piston_extended)

    def toggle_large_piston_position(self):
        if self.large_piston_extended:
            self.set_large_piston_position('retract')
        else:
            self.set_large_piston_position('extend')

    def periodic(self) -> None:
        
        self.counter += 1
        if self.counter % 25 == 1:
            # the compressor turns itself off and on, so we have to ask it its state
            SmartDashboard.putBoolean('compressor_state', self.compressor.enabled())
            # SmartDashboard.putNumber('pressure', self.get_analog_pressure())
            # todo: integrate pressure sensor into compressor class




        





  

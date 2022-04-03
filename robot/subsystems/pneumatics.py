import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, Solenoid, Compressor, AnalogInput


import constants


class Pneumatics(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Pneumatics')
        self.counter = 0

        self.compressor = Compressor(0, wpilib.PneumaticsModuleType.CTREPCM)
        self.intake_piston = Solenoid(wpilib.PneumaticsModuleType.CTREPCM, constants.k_intake_pneumatic_port)
        self.shifter = Solenoid(wpilib.PneumaticsModuleType.CTREPCM, constants.k_shifter_pneumatics_port)
        self.climber_piston_long = Solenoid(wpilib.PneumaticsModuleType.CTREPCM, constants.k_climber_long_port)
        self.climber_piston_short = Solenoid(wpilib.PneumaticsModuleType.CTREPCM, constants.k_climber_short_port)

        self.pressure_sensor = AnalogInput(0)

        self.close_loop_enable = True

        competition = False
        if competition:
            self.start_compressor()
        else:
            self.stop_compressor()

        # Decide on init piston position
        self.intake_extended = False
        SmartDashboard.putBoolean('intake_extended', self.intake_extended)
        SmartDashboard.putBoolean('climber_long_arm', self.climber_piston_long.get())
        SmartDashboard.putBoolean('climber_short_arm', self.climber_piston_short.get())
        SmartDashboard.putBoolean('compressor_close_loop', self.close_loop_enable)
        SmartDashboard.putBoolean('pneumatics_high_gear', self.shifter.get())


    def set_intake_piston(self, position):
        if position == 'extend':
            self.intake_extended = True
            self.intake_piston.set(True)
        elif position == 'retract':
            self.intake_extended = False
            self.intake_piston.set(False)
        SmartDashboard.putBoolean('intake_extended', self.intake_extended)

    def toggle_intake(self):
        self.intake_piston.toggle()
        self.intake_extended = not self.intake_extended
        SmartDashboard.putBoolean('intake_extended', self.intake_extended)

    def get_intake_state(self):
        return self.intake_piston.get()

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

    def up_shift(self):
        self.shifter.set(True)
        SmartDashboard.putBoolean('pneumatics_high_gear', self.shifter.get())

    def down_shift(self):
        self.shifter.set(False)
        SmartDashboard.putBoolean('pneumatics_high_gear', self.shifter.get())

    def toggle_shifting(self):
        self.shifter.toggle()
        SmartDashboard.putBoolean('pneumatics_high_gear', self.shifter.get())
    
    def get_shifting_state(self):
        self.shifter.get()

    def pp_long(self):
        self.climber_piston_long.toggle()
        SmartDashboard.putBoolean('climber_long_arm', self.climber_piston_long.get())
        
    def pp_short(self):
        self.climber_piston_short.toggle()
        SmartDashboard.putBoolean('climber_short_arm', self.climber_piston_short.get())

    def get_analog_pressure(self):
        pressure = 250 * self.pressure_sensor.getVoltage() / 5 - 25
        return pressure

    def periodic(self) -> None:
        
        self.counter += 1

        if self.counter % 10 == 0:
            pressure = self.get_analog_pressure()
            if pressure >= 120:
                self.stop_compressor()
            elif pressure <= 110:
                self.start_compressor()

        if self.counter % 25 == 1:
            # the compressor turns itself off and on, so we have to ask it its state
            SmartDashboard.putBoolean('compressor_state', self.compressor.enabled())
            SmartDashboard.putNumber('pressure', self.get_analog_pressure())




        





  

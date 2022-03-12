import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard, Solenoid, Compressor, DoubleSolenoid


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

        self.close_loop_enable = True

        competition = True
        if competition:
            self.start_compressor()
        else:
            self.stop_compressor()

        # Decide on init piston position
        self.intake_extended = False

    def set_intake_piston(self, position):
        if position == 'extend':
            self.intake_extended = True
            self.intake_piston.set(True)
        elif position == 'retract':
            self.intake_extended = False
            self.intake_piston.set(False)

    def toggle_intake(self):
        self.intake_piston.toggle()
        self.intake_extended = not self.intake_extended

    def get_intake_state(self):
        return self.intake_piston.get()

    def stop_compressor(self):
        # self.compressor.stop()  # deprecated in 2022
        self.compressor.disable()   # should now be disable, as of 2022
        self.close_loop_enable = False

    def start_compressor(self):
        # self.compressor.setClosedLoopControl(True)
        self.compressor.enableDigital()  # setClosedLoopControl(True) is no longer a call in 2022
        self.close_loop_enable = True
    
    def get_compressor_state(self):  # ToDo - fix this - see what Haochen meant, I think he meant 'is it on?'
        # return self.compressor.getClosedLoopControl()
        return self.compressor.enabled()  # getClosedLoopControl is no longer a call in 2022
    
    def up_shift(self):
        self.shifter.set(True)

    def down_shift(self):
        self.shifter.set(False)

    def toggle_shifting(self):
        self.shifter.toggle()
    
    def get_shifting_state(self):
        self.shifter.get()

    def pp_long(self):
        self.climber_piston_long.toggle()
        
    def pp_short(self):
        self.climber_piston_short.toggle()

    def periodic(self) -> None:
        
        self.counter =+ 1

        if self.counter % 50 == 1:
            SmartDashboard.putBoolean('compressor_state', self.get_compressor_state())
            # SmartDashboard.putBoolean('close loop control', self.close_loop_enable)
            SmartDashboard.putBoolean('intake_extended', self.intake_extended)
            SmartDashboard.putBoolean('intake_state', self.intake_piston.get())

        





  

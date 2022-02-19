from commands2 import SubsystemBase
from wpilib import SmartDashboard, Solenoid, Compressor, DoubleSolenoid


import constants


class Pneumatics(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Pneumatics')

        self.compressor = Compressor(0)
        self.intake_piston = Solenoid(constants.k_intake_pneumatic_port)
        self.shifter = Solenoid(constants.k_shifter_pneumatics_port)
        self.climber_piston_long = Solenoid(constants.k_climber_long_port)
        self.climber_piston_short = Solenoid(constants.k_climber_short_port)
        
        #Decide on init piston position 
        self.stop_compressor()

    def toggle_intake(self):
        self.intake_piston.toggle()

    def get_intake_state(self):
        return self.intake_piston.get()

    def stop_compressor(self):
        self.compressor.stop()

    def start_compressor(self):
        self.compressor.setClosedLoopControl(True)
    
    def up_shift(self):
        self.shifter.set(True)

    def down_shift(self):
        self.shifter.set(False)

    def toggle_shifting(self):
        self.shifter.toggle()

    def toggle_climber_long(self):
        self.climber_piston_long.toggle()
    def toggle_climber_short(self):
        self.climber_piston_short.toggle()




    
        


        

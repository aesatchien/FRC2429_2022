import math

import commands2
from wpilib import SmartDashboard



class SpinClimber(commands2.CommandBase):


    def __init__(self, container, climber) -> None:
        super().__init__()
        self.setName('ClimberSpin')
        self.climber = climber
        self.container = container
        self.addRequirements(climber)  # commandsv2 version of requirements
        self.addRequirements(self.container.robot_drive)  # CJH added for safety


    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")


    def execute(self) -> None:
        thrust = self.container.driver_controller.getRawAxis(5)
        scale = 2.0
        power = math.copysign(1, thrust) * (abs(thrust) ** scale) *  12
        self.climber.set_voltage(power)
            

    def isFinished(self) -> bool:  
        # return not self.container.driver_controller.getYButton()
        return False

    def end(self, interrupted: bool) -> None:

        self.climber.stop_motor()

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        

import commands2
from wpilib import SmartDashboard


class LargePistonToggle(commands2.CommandBase):

    def __init__(self, container, pneumatics, force=None) -> None:
        super().__init__()
        self.setName('SmallPistonToggle')
        self.container = container
        self.pneumatics = pneumatics
        self.addRequirements(pneumatics)  # commandsv2 version of requirements
        self.force = force
    def initialize(self) -> None:

        if self.force == 'extend':
            self.pneumatics.set_large_piston_position(position='extend')
        elif self.force == 'retract':
            self.pneumatics.set_large_piston_position(position='retract')
        else:
            self.pneumatics.toggle_large_piston_position()
        
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Firing {self.getName()} with force={self.force} at {self.start_time} s **", flush=True)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:  
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        # print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")


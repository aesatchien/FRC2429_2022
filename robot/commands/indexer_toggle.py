import commands2
from wpilib import SmartDashboard


class ToggleFeed(commands2.CommandBase):


    def __init__(self, container, indexer, voltage=2, force=None) -> None:
        super().__init__()
        self.setName('IndexerToggle')
        self.indexer = indexer
        self.container = container
        self.voltage = voltage
        self.force = force
        self.addRequirements(indexer)  # commandsv2 version of requirements

    def initialize(self) -> None:

        if self.force == 'on':
            self.indexer.set_voltage(self.voltage)
        elif self.force == 'off':
            self.indexer.stop_motor()
        else:
            self.indexer.toggle_indexer(self.voltage)
        
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:  
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        

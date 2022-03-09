import commands2
from wpilib import SmartDashboard



class TimedFeed(commands2.CommandBase):

    def __init__(self, container, indexer, voltage, time=5) -> None:
        super().__init__()
        self.setName('timed feed')
        self.indexer = indexer
        self.container = container
        self.time = time
        self.voltage = voltage
        self.addRequirements(indexer)  # commandsv2 version of requirements

    def initialize(self) -> None:

        self.indexer.set_voltage(self.voltage)
        
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")


        
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:  
        return self.container.get_enabled_time() - self.start_time >= self.time

    def end(self, interrupted: bool) -> None:

        self.indexer.stop_motor()

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)

        

"""
This is an example of a fully defined commandv2 - CJH
Since it is a 'fire and forget' type command, it has empty execute and no isFinished members, so it could be replaced with the commands2.InstantCommand
The initialize also has a message to the console to let us know it fired - good for debugging

Note the only thing that really happens on the robot is that we call the toggle_compressor method in initialize()

"""

import commands2

class CompressorToggle(commands2.CommandBase):

    def __init__(self, container, pneumatics) -> None:
        super().__init__()
        self.setName('CompressorToggle')
        self.pneumatics = pneumatics
        self.container = container
        self.addRequirements(pneumatics)  # commandsv2 version of requirements

    def initialize(self) -> None:

        self.pneumatics.toggle_compressor()
        
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        # SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:  
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        # print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        # SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        

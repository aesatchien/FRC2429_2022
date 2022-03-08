import commands2
from wpilib import SmartDashboard


class ShooterToggle(commands2.CommandBase):

    def __init__(self, container, shooter, rpm=2000, force=None) -> None:
        super().__init__()
        self.setName('toggle shooter')
        self.shooter = shooter
        self.container = container
        self.rpm = rpm
        self.force = force
        self.addRequirements(shooter)  # commandsv2 version of requirements

    def initialize(self) -> None:
        
        if self.force is not None:
            self.shooter.set_flywheel(self.rpm)
        else:
            self.shooter.toggle_shooter(self.rpm)
        
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
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        

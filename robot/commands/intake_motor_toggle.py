
import commands2
from wpilib import SmartDashboard

class IntakeMotorToggle(commands2.CommandBase):
    SmartDashboard.putNumber('intake_speed', 0.65)

    def __init__(self, container, intake, velocity, source=None, force=None) -> None:
        super().__init__()
        self.setName('IntakeMotorToggle')
        self.intake = intake
        self.container = container
        self.velocity = velocity
        self.source = source
        self.force = force
        self.direction = 1
        self.addRequirements(intake)  # commandsv2 version of requirements


    def initialize(self) -> None:
        if self.container.co_driver_controller is not None:
            self.direction = -1 if self.container.co_driver_controller.getStartButton() else 1

        # allow for real-time updates
        if self.source == 'dash':
            self.velocity = SmartDashboard.getNumber('intake_speed', self.velocity)

        if self.force == 'on':
            self.intake.set_velocity(self.velocity)
        elif self.force == 'off':
            self.intake.stop_motor()
        else:
            if not self.intake.intake_enable:  # stop shooter when intaking balls
                self.container.robot_shooter.stop_shooter()
            self.intake.toggle_intake_motor(self.velocity * self.direction)
        
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} with speed {self.velocity} at {self.start_time - self.container.get_enabled_time():2.2f} s **")


    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:  
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        

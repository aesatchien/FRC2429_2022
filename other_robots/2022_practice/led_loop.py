import commands2
from wpilib import SmartDashboard

class LedLoop(commands2.CommandBase):

    def __init__(self, container) -> None:
        super().__init__()
        self.setName('LedLoop')
        self.container = container
        self.addRequirements(container.robot_led)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Firing {self.getName()}  at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        if self.container.robot_shooter.shooter_enable:
            rpm = self.container.robot_shooter.get_flywheel()
            target_rpm = self.container.robot_vision.getShooterRpmNoHood()

            hub_detected = self.container.robot_vision.hub_targets > 0
            aligned = abs(self.container.robot_vision.hub_rotation) < 2

            if hub_detected and aligned and abs(rpm - target_rpm) < 10:
                self.container.robot_led.mode = 'flash'
            else:
                self.container.robot_led.mode = 'loading'
        else:
            self.container.robot_led.mode = 'static'

        self.container.robot_led.spark.set(-1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



import commands2
from wpilib import SmartDashboard

class ClimberRotateSetDistance(commands2.CommandBase):

    def __init__(self, container, climber, setpoint) -> None:
        super().__init__()
        self.setName('ClimberRotateSetDistance')
        self.climber = climber
        self.container = container
        self.setpoint = setpoint

        self.max_vel = 40
        self.max_accel = 500

        self.addRequirements(climber)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.climber.configure_controllers(pid_only=False)

        self.initial_position = self.climber.climber_left_encoder.getPosition()

        self.climber.climber_left_controller.setSmartMotionMaxVelocity(self.max_vel, 0)
        self.climber.climber_left_controller.setSmartMotionMaxAccel(self.max_accel, 0)
        self.climber.smart_motion(self.setpoint, relative=False)

        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        error = self.setpoint - (self.climber.climber_left_encoder.getPosition() - self.initial_position)
        return abs(error) < 0.1

    def end(self, interrupted: bool) -> None:
        self.climber.stop_motor()

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **",
              flush=True)
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



import commands2
from wpilib import SmartDashboard
import rev
import constants

class DriveByJoytick(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive) -> None:
        super().__init__()
        self.setName('drive_by_joystick')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.addRequirements(self.drive)  # commandsv2 version of requirements

        self.max_thrust_velocity = 3  # m/s
        self.max_twist_velocity = 1.5

        self.multipliers = [1.0, 1.0, -1.0, -1.0]

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        thrust = -self.container.driver_controller.getRawAxis(constants.k_controller_thrust_axis) * self.max_thrust_velocity
        twist  = self.container.driver_controller.getRawAxis(constants.k_controller_twist_axis) * self.max_twist_velocity
        self.drive.feed()

        # left front left back  right front  right back
        velocities = [thrust + twist, thrust + twist, thrust - twist, thrust - twist]
        if (thrust**2 + twist**2)**0.5 > 0.05:
            for controller, velocity, multiplier in zip(self.drive.pid_controllers, velocities, self.multipliers):
                controller.setReference(velocity * multiplier, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
        else:
            for controller in self.drive.pid_controllers:
                controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



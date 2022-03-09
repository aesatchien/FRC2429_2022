import math
import commands2
from wpilib import SmartDashboard
import rev
import constants

class DriveByJoytick(commands2.CommandBase):  # change the name for your command

    def __init__(self, container, drive, control_type='velocity', scaling=1) -> None:
        super().__init__()
        self.setName('drive_by_joystick')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.control_type = control_type
        self.scaling = scaling
        self.addRequirements(self.drive)  # commandsv2 version of requirements

        self.max_thrust_velocity = constants.k_max_thrust_velocity  # 2 m/s
        self.max_twist_velocity = constants.k_max_twist_velocity  # 1 m/s
        self.deadband = 0.07
        self.multipliers = [1.0, 1.0, -1.0, -1.0]

        self.max_arcade_thrust = constants.k_thrust_scale
        self.max_arcade_twist = constants.k_twist_scale
        self.previous_thrust = 0

        # since the robot back heavy, so the reverse needs to be stronger than the fwd - rev should be 0.125, fwd 0.25
        self.max_thrust_differential = 0.07  # 0.05 starts to get a bit tippy but not that bad, cory likes 0.04
        self.max_thrust_differential_fwd = 0.05
        self.max_thrust_differential_rev = 0.05

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")


    def execute(self) -> None:
        # get thrust and twist from the controller
        thrust = -(self.container.driver_controller.getRawAxis(constants.k_controller_thrust_axis))
        thrust = 0 if abs(thrust) < self.deadband else math.copysign(1, thrust) * (abs(thrust) ** self.scaling)

        twist = self.container.driver_controller.getRawAxis(constants.k_controller_twist_axis)
        twist = 0 if abs(twist) < self.deadband else math.copysign(1, twist) * (abs(twist) ** self.scaling)

        # try to limit the change in thrust  BE VERY CAREFUL WITH THIS!  IT CAUSES RUNAWAY ROBOTS!
        # limits are made using 3m/s speed limit - adjust if you change it
        limit_decel = True
        if limit_decel:
            d_thrust = self.previous_thrust - thrust
            max_thrust_differential = self.max_thrust_differential_rev if thrust < 0 else self.max_thrust_differential_fwd
            if abs(d_thrust) > max_thrust_differential:
                thrust = self.previous_thrust - max_thrust_differential * math.copysign(1, d_thrust)
                print(f'Applying {1 * math.copysign(1, thrust)} limit: previous thrust: {self.previous_thrust:.3f} delta_thrust: {d_thrust:0.3f}')
        self.previous_thrust = thrust

        if self.control_type == 'velocity':
            self.drive.feed()
            # left front left back  right front  right back
            thrust_vel = thrust * self.max_thrust_velocity
            twist_vel = twist * self.max_twist_velocity
            velocities = [thrust_vel + twist_vel, thrust_vel + twist_vel, thrust_vel - twist_vel, thrust_vel - twist_vel]
            if (thrust**2 + twist**2)**0.5 > 0.05:
                for controller, velocity, multiplier in zip(self.drive.pid_controllers, velocities, self.multipliers):
                    controller.setReference(velocity * multiplier, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
            else:
                for controller in self.drive.pid_controllers:
                    controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)
        else:  # arcade drive
            # fix this - get the thrusts again
            self.drive.arcade_drive(thrust * self.max_arcade_thrust, twist * self.max_arcade_twist)

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        for controller in self.drive.pid_controllers:
            controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 0)


        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



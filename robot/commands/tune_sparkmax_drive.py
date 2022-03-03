import commands2
from wpilib import SmartDashboard
from networktables import NetworkTablesInstance
import rev
from subsystems.drivetrain import Drivetrain
import constants

class TuneSparkmax(commands2.CommandBase):  # change the name for your command

    ntinst = NetworkTablesInstance.getDefault()
    sparkmax_table = ntinst.getTable('Sparkmax')
    keys = ['kP', 'kI', 'kD', 'kIz', 'kFF']
    PID_multiplier = 1000  # small numbers do not show up well on the dash
    for key in keys:
        sparkmax_table.putNumber(key + '_pos', PID_multiplier * constants.PID_dict_pos[key])
        sparkmax_table.putNumber(key + '_vel', PID_multiplier * constants.PID_dict_vel[key])

    def __init__(self, container, drive:Drivetrain, setpoint=1, control_type='position', spin=False) -> None:
        super().__init__()
        self.setName('Tune Sparkmax')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.addRequirements(self.drive)  # commandsv2 version of requirements

        self.tolerance = 0.1  # 10 cm
        self.setpoint = setpoint
        self.control_type = control_type
        self.spin = spin

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.drive.drive.feed()  # keep the watchdog happy

        # update the PIDs in the controllers
        for key in self.keys:
            self.drive.PID_dict_pos[key] = self.sparkmax_table.getNumber(key + '_pos', 0) / self.PID_multiplier
            self.drive.PID_dict_vel[key] = self.sparkmax_table.getNumber(key + '_vel', 0) / self.PID_multiplier
        self.drive.configure_controllers(pid_only=False)

        # set the control type
        if self.control_type == 'positon':
            multipliers = [1.0, 1.0, -1.0, -1.0] if self.spin else [1.0, 1.0, 1.0, 1.0]
            for controller, multiplier in zip(self.drive.pid_controllers, multipliers):
                controller.setReference(self.setpoint * multiplier, rev.CANSparkMaxLowLevel.ControlType.kSmartMotion)
        elif self.control_type == 'velocity':  # velocity needs to be continuous
            multipliers = [1.0, 1.0, -1.0, -1.0] if self.spin else [1.0, 1.0, 1.0, 1.0]
            for controller, multiplier in zip(self.drive.pid_controllers, multipliers):
                controller.setReference(self.setpoint * multiplier, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity)
        else:
            pass

    def execute(self) -> None:
        self.drive.drive.feed()


    def isFinished(self) -> bool:
        if self.control_type == 'velocity':
            return False  # run until cancelled
        elif self.control_type == 'position':
            error = self.setpoint - self.drive.left_encoder.getPosition()
            return abs(self.error) < self.tolerance
        else:
            return True

    def end(self, interrupted: bool) -> None:

        self.drive.tank_drive_volts(0, 0)  # stop the robot

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")



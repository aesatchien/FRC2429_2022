import commands2
from wpimath import controller
from subsystems.pixy import Pixy
from subsystems.drivetrain import Drivetrain
from wpilib import SmartDashboard

# temporary flag for debugging this command, useful for tuning PID controllers
DEBUG_COMMAND = False

# values found by playing with SmartDashboard
DEFAULT_ANGLE_KP = 0.02
DEFAULT_ANGLE_KI = 0.0
DEFAULT_ANGLE_KD = 0.0

# command for seeking ball via vision
class DriveToBall(commands2.CommandBase):
    def __init__(self, pixy: Pixy, drive: Drivetrain, drive_speed: float, angle_kp: float = DEFAULT_ANGLE_KP, angle_ki: float = DEFAULT_ANGLE_KI, angle_kd: float = DEFAULT_ANGLE_KD) -> None:
        super().__init__()

<<<<<<< HEAD
        self.pixy = pixy
        self.drive = drive
        self.controller = controller.PIDController(Kp=0.0, Ki=0.0, Kd=0.0)
        self.addRequirements([self.pixy, self.drive])

    def initialize(self) -> None:
        self.drive.arcadeDrive(0, 0)

        self.drive.resetEncoders()
        self.drive.resetGyro()

    def execute(self) -> None:
        (ball_detected, angle_offset, target_distance) = self.pixy.getBallValues()

        self.controller.setP(SmartDashboard.getNumber('ball_debug/kp', 0))
        self.controller.setI(SmartDashboard.getNumber('ball_debug/ki', 0))
        self.controller.setD(SmartDashboard.getNumber('ball_debug/kd', 0))

        if ball_detected:
            current_angle = self.drive.gyro.getAngleZ()
            pid_output = self.controller.calculate(current_angle, current_angle + angle_offset)
            self.drive.arcadeDrive(0, pid_output)

            SmartDashboard.putNumber('ball_debug/pid_output', pid_output)
            SmartDashboard.putNumber('ball_debug/angle_z', current_angle)
=======
        # command uses the pixy for vision input and drivetrain for movement
        self.pixy = pixy
        self.drive = drive
        self.addRequirements([self.pixy, self.drive])

        # initialize PID controllers
        self.angle_controller = controller.PIDController(0, 0, 0)
        if DEBUG_COMMAND:
            # debug PID values via SmartDashboard
            SmartDashboard.putBoolean('DriveToBall/debugging', DEBUG_COMMAND)

            SmartDashboard.putNumber('DriveToBall/kp', angle_kp);
            SmartDashboard.putNumber('DriveToBall/ki', angle_ki);
            SmartDashboard.putNumber('DriveToBall/kd', angle_kd);
            SmartDashboard.putNumber('DriveToBall/drive_speed', drive_speed)
        else:
            self.angle_controller.setP(angle_kp)
            self.angle_controller.setI(angle_ki)
            self.angle_controller.setD(angle_kd)

        self.drive_speed = drive_speed

    def initialize(self) -> None:
        self.drive.arcadeDrive(0, 0)
        self.drive.resetEncoders()
        self.drive.resetGyro()

        # initialize PID controller setpoint
        (ball_detected, angle_offset, _) = self.pixy.getBallValues()
        if ball_detected:
            current_angle = self.drive.gyro.getAngleZ()

            # pixy gives a relative offset in [-30, +30]
            self.angle_controller.setSetpoint(current_angle + angle_offset)

    def execute(self) -> None:
        if DEBUG_COMMAND:
            self.angle_controller.setP(SmartDashboard.getNumber('DriveToBall/kp', DEFAULT_ANGLE_KP))
            self.angle_controller.setI(SmartDashboard.getNumber('DriveToBall/ki', DEFAULT_ANGLE_KI))
            self.angle_controller.setD(SmartDashboard.getNumber('DriveToBall/kd', DEFAULT_ANGLE_KD))
            self.drive_speed = SmartDashboard.getNumber('DriveToBall/drive_speed', 0)

        (ball_detected, angle_offset, distance) = self.pixy.getBallValues()

        if ball_detected:
            # if ball is detected, drive towards it at constant speed
            current_angle = self.drive.gyro.getAngleZ()
            pid_output = self.angle_controller.calculate(current_angle, current_angle - angle_offset)

            self.drive.arcadeDrive(self.drive_speed, pid_output)
>>>>>>> origin/romi_hw

    def end(self, interrupted: bool) -> None:
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
<<<<<<< HEAD
=======
        # drive to ball as long as button is held
        # make command finish once it is close enough to ball?
>>>>>>> origin/romi_hw
        return False
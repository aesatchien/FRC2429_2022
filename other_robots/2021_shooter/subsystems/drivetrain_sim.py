#drivetrain to use if we are in simulation mode

import wpimath.kinematics
import wpilib.drive
from wpilib.command import Subsystem
from wpilib import SmartDashboard, SpeedControllerGroup, Timer
import wpimath.geometry as geo

import subsystems.drive_constants as drive_constants
from commands.drive_by_joystick import DriveByJoystick
import navx
import rev

class DriveTrainSim(Subsystem):
    # ----------------- INITIALIZATION -----------------------
    def __init__(self, robot):
        super().__init__("drivetrainsim")
        self.robot = robot
        self.counter = 0  # used for updating the log
        self.x, self.y = 0, 0

        # initialize sensors
        self.navx = navx.AHRS.create_spi()

        # initialize motors
        self.spark_neo_left_front = wpilib.Jaguar(1)
        self.spark_neo_left_rear = wpilib.Jaguar(2)
        self.spark_neo_right_front = wpilib.Jaguar(3)
        self.spark_neo_right_rear = wpilib.Jaguar(4)

        motor_type = rev.MotorType.kBrushless
        self.spark_neo_left_front1 = rev.CANSparkMax(1, motor_type)
        self.spark_neo_left_rear1 = rev.CANSparkMax(2, motor_type)
        self.spark_neo_right_front1 = rev.CANSparkMax(3, motor_type)
        self.spark_neo_right_rear1 = rev.CANSparkMax(4, motor_type)
        self.sparkneo_encoder_1 = rev.CANSparkMax.getEncoder(self.spark_neo_left_front1)
        self.sparkneo_encoder_2 = rev.CANSparkMax.getEncoder(self.spark_neo_left_rear1)
        self.sparkneo_encoder_3 = rev.CANSparkMax.getEncoder(self.spark_neo_right_front1)
        self.sparkneo_encoder_4 = rev.CANSparkMax.getEncoder(self.spark_neo_right_rear1)

        # create drivetrain from motors
        self.speedgroup_left = SpeedControllerGroup(self.spark_neo_left_front, self.spark_neo_left_rear)
        self.speedgroup_right = SpeedControllerGroup(self.spark_neo_right_front, self.spark_neo_right_rear)
        self.drive = wpilib.drive.DifferentialDrive(self.speedgroup_left, self.speedgroup_right)
        # self.drive = wpilib.drive.DifferentialDrive(self.spark_neo_left_front, self.spark_neo_right_front)
        self.drive.setMaxOutput(1.0)

        # initialize encoders - doing this after motors because they may be part of the motor controller
        self.l_encoder = wpilib.Encoder(0, 1, True)
        self.r_encoder = wpilib.Encoder(2, 3, True)
        self.l_encoder.setDistancePerPulse(drive_constants.k_encoder_distance_per_pulse_m)
        self.r_encoder.setDistancePerPulse(drive_constants.k_encoder_distance_per_pulse_m)

        # odometry for tracking the robot pose
        self.odometry = wpimath.kinematics.DifferentialDriveOdometry(geo.Rotation2d.fromDegrees( -self.navx.getAngle() ))

    def initDefaultCommand(self):
        """ When other commands aren't using the drivetrain, allow arcade drive with the joystick. """
        self.setDefaultCommand(DriveByJoystick(self.robot))

    # ----------------- DRIVE METHODS -----------------------
    def arcade_drive(self, thrust, twist):
        """ wrapper for the current drive mode, really should just be called drive or move """
        self.drive.arcadeDrive(xSpeed=thrust, zRotation=twist, squareInputs=True)

    def stop(self):
        """ stop the robot """
        self.drive.arcadeDrive(xSpeed=0, zRotation=0, squareInputs=True)

    # ----------------- SIMULATION AND TELEMETRY METHODS -----------------------
    def get_pose(self):
        # return self.odometry.getPoseMeters() # 2021 only?
        return self.odometry.getPose()

    def get_wheel_speeds(self):
        return wpimath.kinematics.DifferentialDriveWheelSpeeds(self.l_encoder.getRate(), self.r_encoder.getRate())

    def reset_encoders(self):
        self.l_encoder.reset()
        self.r_encoder.reset()

    def get_rate(self, encoder): # spark maxes and regular encoders use different calls... annoying
        return encoder.getRate()
    def get_position(self, encoder):
        return encoder.getDistance()
    def set_position(self, encoder, position):
        encoder.setDistance(position)

    def reset_odometry(self, pose):
        self.reset_encoders()
        self.odometry.resetPosition(pose, geo.Rotation2d.fromDegrees(-self.navx.getAngle()))

    def tank_drive_volts(self, left_volts, right_volts):
        self.speedgroup_left.setVoltage(left_volts)
        self.speedgroup_right.setVoltage(right_volts)
        self.drive.feed()

    def get_rotation2d(self):
        return geo.Rotation2d.fromDegrees(-self.navx.getAngle())

    def get_average_encoder_distance(self):
        return (self.l_encoder.getDistance() - self.r_encoder.getDistance())/2  #  used in PID drive straight

    def get_average_encoder_rate(self):
        return (self.l_encoder.getRate() + self.r_encoder.getRate())/2

    def zero_heading(self):
        self.navx.reset()

    def reset(self):
        pass

    def periodic(self) -> None:
        """Perform odometry and update dash with telemetry"""
        self.counter += 1
        self.odometry.update(geo.Rotation2d.fromDegrees(-self.navx.getAngle()), self.l_encoder.getDistance(), -self.r_encoder.getDistance())

        if self.counter % 10 == 0:
            # start keeping track of where the robot is with an x and y position (only good for WCD)'
            pose = self.get_pose()
            SmartDashboard.putString('drive_pose', f'[{pose.X():2.2f}, {pose.Y():2.2f}, {pose.rotation().degrees():2.2f}]' )

        if self.counter % 100 == 0:
            pass
            # self.display_PIDs()
            # SmartDashboard.putString("alert", f"Position: ({round(self.x, 1)},{round(self.y, 1)})  Time: {round(Timer.getFPGATimestamp() - self.robot.enabled_time, 1)}")
            # print(self.get_wheel_speeds(), self.l_encoder.getRate(), self.r_encoder.getRate())

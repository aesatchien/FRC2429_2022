# drivetrain to use both in sim and robot mode - sim handles the Sparkmax now

import wpimath.kinematics
import wpilib.drive
from wpilib.command import Subsystem
from wpilib import SmartDashboard, SpeedControllerGroup, Timer
import wpimath.geometry as geo

import subsystems.drive_constants as drive_constants
from commands.drive_by_joystick import DriveByJoystick
import navx
import rev

class DriveTrain(Subsystem):
    # ----------------- INITIALIZATION -----------------------
    def __init__(self, robot):
        super().__init__("drivetrain")
        self.robot = robot
        self.counter = 0  # used for updating the log
        self.x, self.y = 0, 0
        self.error_dict = {}

        # initialize sensors
        self.navx = navx.AHRS.create_spi()

        # initialize motors and encoders
        # motor_type = rev.MotorType.kBrushless
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless

        #self.spark_neo_left_front = rev.CANSparkMax(1, motor_type)
        #self.spark_neo_left_rear = rev.CANSparkMax(2, motor_type)
        #self.spark_neo_right_front = rev.CANSparkMax(3, motor_type)
        #self.spark_neo_right_rear = rev.CANSparkMax(4, motor_type)
        self.spark_neo_left_front = rev.CANSparkMax(4, motor_type) # switched left and right
        self.spark_neo_left_rear = rev.CANSparkMax(3, motor_type)
        self.spark_neo_right_front = rev.CANSparkMax(2, motor_type)
        self.spark_neo_right_rear = rev.CANSparkMax(1, motor_type)
        self.controllers = [self.spark_neo_left_front, self.spark_neo_left_rear,
                            self.spark_neo_right_front, self.spark_neo_right_rear]

        # self.spark_neo_right_rear.setInverted(False)
        # self.spark_neo_right_front.setInverted(False)

        self.spark_PID_controller_right_front = self.spark_neo_right_front.getPIDController()
        self.spark_PID_controller_right_rear = self.spark_neo_right_rear.getPIDController()
        self.spark_PID_controller_left_front = self.spark_neo_left_front.getPIDController()
        self.spark_PID_controller_left_rear = self.spark_neo_left_rear.getPIDController()
        self.pid_controllers = [self.spark_PID_controller_left_front, self.spark_PID_controller_left_rear,
                                self.spark_PID_controller_right_front, self.spark_PID_controller_right_rear]

        # swap encoders to get sign right
        # changing them up for mecanum vs WCD
        self.sparkneo_encoder_1 = rev.CANSparkMax.getEncoder(self.spark_neo_left_front)
        self.sparkneo_encoder_2 = rev.CANSparkMax.getEncoder(self.spark_neo_left_rear)
        self.sparkneo_encoder_3 = rev.CANSparkMax.getEncoder(self.spark_neo_right_front)
        self.sparkneo_encoder_4 = rev.CANSparkMax.getEncoder(self.spark_neo_right_rear)
        self.encoders = [self.sparkneo_encoder_1, self.sparkneo_encoder_2, self.sparkneo_encoder_3, self.sparkneo_encoder_4]
        # copy these so the sim and the real reference the same encoders
        self.l_encoder = self.sparkneo_encoder_1
        self.r_encoder = self.sparkneo_encoder_3
        # Configure encoders and controllers
        # should be wheel_diameter * pi / gear_ratio - and for the old double reduction gear box
        # the gear ratio was 4.17:1.  With the shifter (low gear) I think it was a 12.26.
        # the new 2020 gearbox is 9.52'
        #gear_ratio = 12.75
        #gear_ratio = 4.17  # pretty fast WCD gearbox
        #conversion_factor = 6.0 * 0.0254 * 3.1416 / gear_ratio  # do this in meters from now on
        conversion_factor = drive_constants.k_sparkmax_conversion_factor_meters
        for ix, encoder in enumerate(self.encoders): 
            self.error_dict.update({'conv_pos_'+ str(ix): encoder.setPositionConversionFactor(conversion_factor)})
            self.error_dict.update({'conv_vel_' + str(ix): encoder.setVelocityConversionFactor(conversion_factor/60.0)})  # native is rpm
        burn_flash = False
        if burn_flash:
            for ix, controller in enumerate(self.controllers):
                self.error_dict.update({'burn_'+ str(ix): controller.burnFlash()})

        # TODO - figure out if I want to invert the motors or the encoders
        #inverted = False  # needs this to be True for the toughbox
        #self.spark_neo_left_front.setInverted(inverted)  # inverting a controller
        #self.r_encoder.setInverted(True)  # inverting an encoder


        # create drivetrain from motors
        self.speedgroup_left = SpeedControllerGroup(self.spark_neo_left_front, self.spark_neo_left_rear)
        self.speedgroup_right = SpeedControllerGroup(self.spark_neo_right_front, self.spark_neo_right_rear)
        self.drive = wpilib.drive.DifferentialDrive(self.speedgroup_left, self.speedgroup_right)
        # self.drive = wpilib.drive.DifferentialDrive(self.spark_neo_left_front, self.spark_neo_right_front)
        self.drive.setMaxOutput(1.0)
        self.drive.setSafetyEnabled(True)
        self.drive.setExpiration(0.1)


        # odometry for tracking the robot pose
        self.odometry = wpimath.kinematics.DifferentialDriveOdometry(geo.Rotation2d.fromDegrees( -self.navx.getAngle() ))

    def initDefaultCommand(self):
        """ When other commands aren't using the drivetrain, allow arcade drive with the joystick. """
        self.setDefaultCommand(DriveByJoystick(self.robot))

    # ----------------- DRIVE METHODS -----------------------
    def arcade_drive(self, thrust, twist):
        """ wrapper for the current drive mode, really should just be called drive or move """
        self.drive.arcadeDrive(xSpeed=thrust, zRotation=twist, squareInputs=True)
        #[controller.setVoltage(thrust) for controller in self.controllers]
        #self.drive.feed()

    def stop(self):
        """ stop the robot """
        self.drive.arcadeDrive(xSpeed=0, zRotation=0, squareInputs=True)

    # ----------------- SIMULATION AND TELEMETRY METHODS -----------------------
    def get_pose(self):
        # return self.odometry.getPoseMeters() # 2021 only?
        return self.odometry.getPose()

    def get_wheel_speeds(self):
        return wpimath.kinematics.DifferentialDriveWheelSpeeds(self.l_encoder.getVelocity(), self.r_encoder.getVelocity())

    def reset_encoders(self):
        self.l_encoder.setPosition(0)
        self.r_encoder.setPosition(0)

    def get_rate(self, encoder): # spark maxes and regular encoders use different calls... annoying
        return encoder.getVelocity()

    def get_position(self, encoder):
        return encoder.getPosition()

    def set_position(self, encoder, position):
        encoder.setPosition(position)

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
        return (self.l_encoder.getPosition() - self.r_encoder.getPosition())/2

    def get_average_encoder_rate(self):
        return (self.l_encoder.getVelocity() + self.r_encoder.getVelocity())/2

    def zero_heading(self):
        self.navx.reset()

    def reset(self):
        self.zero_heading()
        self.reset_encoders()


    def periodic(self) -> None:
        """Perform odometry and update dash with telemetry"""
        self.counter += 1
        self.odometry.update(geo.Rotation2d.fromDegrees(-self.navx.getAngle()), self.l_encoder.getPosition(), -self.r_encoder.getPosition())

        if self.counter % 10 == 0:
            # start keeping track of where the robot is with an x and y position (only good for WCD)'
            pose = self.get_pose()
            SmartDashboard.putString('drive_pose', f'[{pose.X():2.2f}, {pose.Y():2.2f}, {pose.rotation().degrees():2.2f}]' )
            SmartDashboard.putString('drive_encoders LR',
                                     f'[{self.get_position(self.l_encoder):2.3f}, {self.get_position(self.r_encoder):2.2f}]')
            SmartDashboard.putString('drive_heading',
                                     f'[{-self.navx.getAngle():2.3f}]')

        if self.counter % 100 == 0:
            pass
            # self.display_PIDs()
            msg = f"Positions: ({self.l_encoder.getPosition():2.2f}, {self.r_encoder.getPosition():2.2}, {self.navx.getAngle():2.2})"
            msg = msg + f" Rates: ({self.l_encoder.getVelocity():2.2f}, {self.r_encoder.getVelocity():2.2f})  Time: {Timer.getFPGATimestamp() - self.robot.enabled_time:2.1f}"
            SmartDashboard.putString("alert", msg)
            SmartDashboard.putString("sparks", str(self.error_dict))
            # print(self.get_wheel_speeds(), self.l_encoder.getRate(), self.r_encoder.getRate())

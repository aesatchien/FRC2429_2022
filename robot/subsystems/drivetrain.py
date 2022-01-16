# drivetrain to use both in sim and robot mode - sim handles the Sparkmax now
# started 2022 0102 to update to commands2

from commands2 import SubsystemBase
from wpilib import SpeedControllerGroup, PWMSparkMax, SmartDashboard
from wpilib.drive import DifferentialDrive
from wpimath.kinematics import DifferentialDriveOdometry, DifferentialDriveWheelSpeeds
import wpimath.geometry as geo
import rev
import navx

import constants


class Drivetrain(SubsystemBase):
    def __init__(self):
        super().__init__()

        self.counter = 0  # used for updating the log
        self.x, self.y = 0, 0
        self.error_dict = {}  # used for tracking errors from the SparkMAX controllers

        # initialize sensors - use the navx for headings
        self.navx = navx.AHRS.create_spi()

        # initialize motors
        motor_type = rev.MotorType.kBrushless
        self.spark_neo_left_front = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_left_rear = rev.CANSparkMax(constants.k_left_motor2_port, motor_type)
        self.spark_neo_right_front = rev.CANSparkMax(constants.k_right_motor1_port, motor_type)
        self.spark_neo_right_rear = rev.CANSparkMax(constants.k_right_motor2_port, motor_type)
        self.controllers = [self.spark_neo_left_front, self.spark_neo_left_rear,
                            self.spark_neo_right_front, self.spark_neo_right_rear]

        # add two dummy PWMs so we can track the SparkMax in the sim (updated in periodic)
        self.dummy_motor_left = PWMSparkMax(1)
        self.dummy_motor_right = PWMSparkMax(3)

        # Create the motor controllers and their respective speed controllers
        self.left_motors = SpeedControllerGroup(self.spark_neo_left_front, self.spark_neo_left_rear)
        self.right_motors = SpeedControllerGroup(self.spark_neo_right_front, self.spark_neo_right_rear)

        # Create the differential drivetrain object, allowing for easy motor control
        self.drive = DifferentialDrive(self.left_motors, self.right_motors)
        self.drive.setMaxOutput(1.0)  # default, not necessary but here for training students
        self.drive.setSafetyEnabled(True)  # default, not necessary but here for training students
        self.drive.setExpiration(0.1)

        # Create the encoder objects
        self.spark_neo_encoder_1 = rev.CANSparkMax.getEncoder(self.spark_neo_left_front)
        self.spark_neo_encoder_2 = rev.CANSparkMax.getEncoder(self.spark_neo_left_rear)
        self.spark_neo_encoder_3 = rev.CANSparkMax.getEncoder(self.spark_neo_right_front)
        self.spark_neo_encoder_4 = rev.CANSparkMax.getEncoder(self.spark_neo_right_rear)
        self.encoders = [self.spark_neo_encoder_1, self.spark_neo_encoder_2, self.spark_neo_encoder_3, self.spark_neo_encoder_4]

        # copy these so the sim and the real reference the same encoders  (just a reference, not a copy)
        self.left_encoder = self.spark_neo_encoder_1
        self.right_encoder = self.spark_neo_encoder_3

        # Configure encoders
        conversion_factor = constants.k_sparkmax_conversion_factor_meters
        for ix, encoder in enumerate(self.encoders):
            self.error_dict.update({'conv_pos_' + str(ix): encoder.setPositionConversionFactor(conversion_factor)})
            self.error_dict.update({'conv_vel_' + str(ix): encoder.setVelocityConversionFactor(conversion_factor / 60.0)})  # native is rpm

        # Create the object for our odometry, which will utilize sensor data to keep a record of our position on the field
        self.odometry = DifferentialDriveOdometry(geo.Rotation2d.fromDegrees(-self.navx.getAngle()))

        # Reset the encoders upon the initialization of the robot
        self.reset_encoders()

    # ----------------- SIMULATION AND TELEMETRY METHODS -----------------------
    def get_pose(self):
        """Returns the current position of the robot using its odometry."""
        return self.odometry.getPose()

    def get_wheel_speeds(self):
        """Return an object which represents the wheel speeds of our drivetrain."""
        speeds = DifferentialDriveWheelSpeeds(self.left_encoder.getVelocity(), self.right_encoder.getVelocity())
        return speeds

    def reset_odometry(self, pose):
        """ Resets the robot's odometry to a given position."""
        self.reset_encoders()
        self.odometry.resetPosition(pose, self.navx.getRotation2d())

    def arcade_drive(self, fwd, rot):
        """Drive the robot with standard arcade controls."""
        self.drive.arcadeDrive(fwd, rot)
        self.dummy_motor_left.set(self.spark_neo_left_front.get())
        self.dummy_motor_right.set(self.spark_neo_right_front.get())

    def tank_drive_volts(self, left_volts, right_volts):
        """Control the robot's drivetrain with voltage inputs for each side."""
        # Set the voltage of the left side.
        self.left_motors.setVoltage(left_volts)
        # Set the voltage of the right side. It's inverted with a negative sign
        #  because its motors need to spin in the negative direction to move forward
        self.right_motors.setVoltage(right_volts)

        self.dummy_motor_left.set(left_volts/12)
        self.dummy_motor_right.set(right_volts/12)
        SmartDashboard.putNumber('/drive/left_volts', left_volts)
        SmartDashboard.putNumber('/drive/right_volts', right_volts)

        # Resets the timer for this motor's MotorSafety
        self.drive.feed()

    def reset_encoders(self):
        """Resets the encoders of the drivetrain."""
        self.left_encoder.setPosition(0)
        self.right_encoder.setPosition(0)

    def get_average_encoder_distance(self):
        """
        Take the sum of each encoder's traversed distance and divide it by two,
        since we have two encoder values, to find the average value of the two.
        """
        return (self.left_encoder.getPosition() + self.right_encoder.getPosition()) / 2

    def get_left_encoder(self):
        """Returns the left encoder object."""
        return self.left_encoder

    def get_right_encoder(self):
        """Returns the right encoder object."""
        return self.right_encoder

    def set_max_output(self, max_output):
        """Set the max percent output of the drivetrain, allowing for slower control."""
        self.drive.setMaxOutput(max_output)

    def zero_heading(self):
        """Zeroes the navx's heading."""
        self.navx.reset()

    def get_heading(self):
        """Return the current heading of the robot."""
        # return self.gyro.getRotation2d().getDegrees()
        return -self.navx.getAngle()

    def get_rate(self, encoder): # spark maxes and regular encoders use different calls... annoying
        return encoder.getVelocity()

    def get_average_encoder_rate(self):
        return (self.left_encoder.getVelocity() + self.right_encoder.getVelocity())/2

    def get_rotation2d(self):
        return geo.Rotation2d.fromDegrees(-self.navx.getAngle())

    def get_turn_rate(self):
        """Returns the turning rate of the robot using the navx."""
        # The minus sign negates the value.
        #return -self.gyro.getRate()
        return -self.navx.getRate()

    def reset(self):
        self.zero_heading()
        self.reset_encoders()

    # ----------------- PERIODIC UPDATES -----------------------
    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's odometry with sensor data.
        """
        self.counter += 1
        self.odometry.update(geo.Rotation2d.fromDegrees(-self.navx.getAngle()),
                             self.left_encoder.getPosition(), -self.right_encoder.getPosition())

        if self.counter % 10 == 0:
            # start keeping track of where the robot is with an x and y position (only good for WCD)'
            pose = self.get_pose()
            SmartDashboard.putString('/drive/drive_pose', f'[{pose.X():2.2f}, {pose.Y():2.2f}, {pose.rotation().degrees():2.2f}]' )

        if self.counter % 100 == 0:
            pass
            # self.display_PIDs()
            # msg = f"Positions: ({self.l_encoder.getPosition():2.2f}, {self.r_encoder.getPosition():2.2}, {self.navx.getAngle():2.2})"
            # msg = msg + f" Rates: ({self.l_encoder.getVelocity():2.2f}, {self.r_encoder.getVelocity():2.2f})  Time: {Timer.getFPGATimestamp() - self.robot.enabled_time:2.1f}"
            # SmartDashboard.putString("alert", msg)
            # SmartDashboard.putString("sparks", str(self.error_dict))
            # print(self.get_wheel_speeds(), self.l_encoder.getRate(), self.r_encoder.getRate())

    def simulationPeriodic(self) -> None:
        pass
        #self.dummy_motor_left.set(self.spark_neo_left_front.get())
        #self.dummy_motor_right.set(self.spark_neo_right_front.get())
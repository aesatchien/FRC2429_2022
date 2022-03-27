# drivetrain to use both in sim and robot mode - sim handles the Sparkmax now
# started 2022 0102 to update to commands2
import math
import time
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
        self.previous_vel = 0  # used for simulating acceleration
        self.previous_time = time.time()

        # if we need to configure the sparkmaxes for smartvelocity and smartmotion
        self.error_dict = {}  # used for tracking errors from the SparkMAX controllers
        self.PID_dict_pos = constants.PID_dict_pos
        self.PID_dict_vel = constants.PID_dict_vel
        self.smartmotion_maxvel = constants.smartmotion_maxvel
        self.smartmotion_maxacc = constants.smartmotion_maxacc
        self.current_limit = constants.current_limit
        self.ramp_rate = constants.ramp_rate

        """ self.PID_dict_pos = {'kP': 0.010, 'kI': 5.0e-7, 'kD': 0.40, 'kIz': 0, 'kFF': 0.002, 'kMaxOutput': 0.99, 'kMinOutput': -0.99}
        self.PID_dict_vel = {'kP': 2*0.00015, 'kI': 1.5*8.0e-7, 'kD': 0.00, 'kIz': 0, 'kFF': 0.00022, 'kMaxOutput': 0.99, 'kMinOutput': -0.99}
        self.smartmotion_maxvel = 1000  # rpm
        self.smartmotion_maxacc = 500
        self.current_limit = 100
        self.ramp_rate = 0.0"""

        # initialize sensors - use the navx for headings
        self.navx = navx.AHRS.create_spi()

        # initialize motors
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.spark_neo_left_front = rev.CANSparkMax(constants.k_left_motor1_port, motor_type)
        self.spark_neo_left_rear = rev.CANSparkMax(constants.k_left_motor2_port, motor_type)
        self.spark_neo_right_front = rev.CANSparkMax(constants.k_right_motor1_port, motor_type)
        self.spark_neo_right_rear = rev.CANSparkMax(constants.k_right_motor2_port, motor_type)
        self.controllers = [self.spark_neo_left_front, self.spark_neo_left_rear,
                            self.spark_neo_right_front, self.spark_neo_right_rear]

        # get PID controllers
        self.spark_PID_controller_right_front = self.spark_neo_right_front.getPIDController()
        self.spark_PID_controller_right_rear = self.spark_neo_right_rear.getPIDController()
        self.spark_PID_controller_left_front = self.spark_neo_left_front.getPIDController()
        self.spark_PID_controller_left_rear = self.spark_neo_left_rear.getPIDController()
        self.pid_controllers = [self.spark_PID_controller_left_front, self.spark_PID_controller_left_rear,
                                self.spark_PID_controller_right_front, self.spark_PID_controller_right_rear]
        
        # 2022 bot w/ 2021 robotpy: shifter gearboxes are CCW when motors are CW.  So we invert all four.
        # However, on 2022 wpilib, the right side of differential drive will no longer be inverted by default
        [controller.setInverted(True) for controller in self.controllers]  # approach for shifter GB

        # add two dummy PWMs so we can track the SparkMax in the sim (should be updated in sim periodic)
        self.dummy_motor_left = PWMSparkMax(1)
        self.dummy_motor_right = PWMSparkMax(3)

        # Create the motor controllers and their respective speed controllers
        self.left_motors = SpeedControllerGroup(self.spark_neo_left_front, self.spark_neo_left_rear)
        self.right_motors = SpeedControllerGroup(self.spark_neo_right_front, self.spark_neo_right_rear)
        self.right_motors.setInverted(True)  # 2022 change - drivetrain used to invert this by default before 2022

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
        # self.odometry = DifferentialDriveOdometry(geo.Rotation2d.fromDegrees(-self.navx.getAngle()))

        # Reset the encoders upon the initialization of the robot
        self.reset_encoders()

        self.navx.setAngleAdjustment(constants.k_start_heading - self.navx.getAngle())
        time.sleep(0.01)  # how long does it take to reset the navX?
        self.odometry = DifferentialDriveOdometry(gyroAngle=geo.Rotation2d.fromDegrees(-self.navx.getAngle()), initialPose=geo.Pose2d(constants.k_start_x, constants.k_start_y,
                                                                         geo.Rotation2d.fromDegrees(constants.k_start_heading)))

        # set us on the board where we want to be in simulation
        if constants.k_is_simulation:
            # pass
            self.reset_odometry(pose=geo.Pose2d(constants.k_start_x, constants.k_start_y, geo.Rotation2d.fromDegrees(constants.k_start_heading)))
            # self.odometry = DifferentialDriveOdometry(initialPose=geo.Pose2d(constants.k_start_x, constants.k_start_y, geo.Rotation2d.fromDegrees(constants.k_start_heading)))
        else:
            pass
            # self.reset_odometry(pose=geo.Pose2d(constants.k_start_x,
            #                     constants.k_start_y, geo.Rotation2d().fromDegrees(constants.k_start_heading)))

        # configure the controllers
        self.configure_controllers(pid_only=False, burn_flash=False)

    # ----------------- SIMULATION AND TELEMETRY METHODS -----------------------
    def get_pose(self):  # used in ramsete and in this subsystem's updates
        """Returns the current position of the robot using its odometry."""
        return self.odometry.getPose()

    def get_wheel_speeds(self):  # used in ramsete
        """Return an object which represents the wheel speeds of our drivetrain."""
        speeds = DifferentialDriveWheelSpeeds(self.left_encoder.getVelocity(), self.right_encoder.getVelocity())
        return speeds

    def reset_odometry(self, pose):  # used in ramsete
        """ Resets the robot's odometry to a given position."""
        print(f'resetting odometry to {pose}')
        self.reset_encoders()
        self.navx.setAngleAdjustment(-constants.k_start_heading - self.navx.getYaw())
        #self.navx.setAngleAdjustment(constants.k_start_heading - self.navx.getAngle())
        self.odometry = DifferentialDriveOdometry(gyroAngle=pose.rotation(), initialPose=pose)

    def arcade_drive(self, fwd, rot):
        """Drive the robot with standard arcade controls."""
        self.drive.arcadeDrive(fwd, rot)

        # need to update the simulated PWMs here
        self.dummy_motor_left.set(self.spark_neo_left_front.get())
        self.dummy_motor_right.set(self.spark_neo_right_front.get())

    def tank_drive_volts(self, left_volts, right_volts):
        """Control the robot's drivetrain with voltage inputs for each side."""
        # Set the voltage of the left side.
        self.left_motors.setVoltage(left_volts)
        # Set the voltage of the right side. It's inverted with a negative sign
        #  because its motors need to spin in the negative direction to move forward
        self.right_motors.setVoltage(right_volts)

        # need to update the simulated PWMs here
        self.dummy_motor_left.set(left_volts/12)
        self.dummy_motor_right.set(-right_volts/12)
        SmartDashboard.putNumber('/drive/left_volts', left_volts)
        SmartDashboard.putNumber('/drive/right_volts', right_volts)

        # Resets the timer for this motor's MotorSafety
        self.drive.feed()

    def reset_encoders(self):  # part of resetting odometry
        """Resets the encoders of the drivetrain."""
        self.left_encoder.setPosition(0)
        time.sleep(0.05)
        self.feed()
        self.right_encoder.setPosition(0)
        time.sleep(0.05)
        self.feed()

    def get_average_encoder_distance(self):  # never used
        """
        Take the sum of each encoder's traversed distance and divide it by two,
        since we have two encoder values, to find the average value of the two.
        """
        return (self.left_encoder.getPosition() + self.right_encoder.getPosition()) / 2

    def get_left_encoder(self):  # used in ramsete
        """Returns the left encoder object."""
        return self.left_encoder

    def get_right_encoder(self):  # used in ramsete
        """Returns the right encoder object."""
        return self.right_encoder

    def set_max_output(self, max_output):  # used to scale all outputs down, used in the button lambda
        """Set the max percent output of the drivetrain, allowing for slower control."""
        self.drive.setMaxOutput(max_output)

    def zero_heading(self):  # only used in reset drivetrain?
        """Zeroes the navx's heading."""
        self.navx.reset()

    def get_heading(self):  # never used
        """Return the current heading of the robot."""
        # return self.gyro.getRotation2d().getDegrees()
        return -self.navx.getAngle()
        # negative makes you increase CCW, as Ramsete wants.  Do the 360 remainder or it flips sign at 180.
        # return - math.remainder(self.navx.Angle(), 360)


    def get_rate(self, encoder): # spark maxes and regular encoders use different calls... annoying.  used in ramsete.
        return encoder.getVelocity()

    def get_position(self, encoder):
        return encoder.getPosition()

    def get_average_encoder_rate(self):  # used in ramsete
        return (self.left_encoder.getVelocity() - self.right_encoder.getVelocity())/2

    def get_rotation2d(self):  # used in ramsete
        return geo.Rotation2d.fromDegrees(-self.navx.getAngle())

    def get_turn_rate(self):  # never used
        """Returns the turning rate of the robot using the navx."""
        # The minus sign negates the value.
        #return -self.gyro.getRate()
        return -self.navx.getRate()

    def reset(self):
        self.zero_heading()
        self.reset_encoders()
    
    def feed(self):
        self.drive.feed()

    # ----------------- PERIODIC UPDATES -----------------------
    def periodic(self):
        """
        Called periodically when it can be called. Updates the robot's odometry with sensor data.
        """
        self.counter += 1
        self.odometry.update(geo.Rotation2d.fromDegrees(-self.navx.getAngle()),
                             self.left_encoder.getPosition(), -self.right_encoder.getPosition())

        if self.counter % 15 == 0:
            # start keeping track of where the robot is with an x and y position (only good for WCD)'
            pose = self.get_pose()
            # SmartDashboard.putString('drive_pose', f'[{pose.X():2.2f}, {pose.Y():2.2f}, {pose.rotation().degrees():2.2f}]' )
            SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
            SmartDashboard.putNumber('drive_lpos', self.left_encoder.getPosition())
            SmartDashboard.putNumber('drive_rpos', self.right_encoder.getPosition())
            SmartDashboard.putNumber('drive_lvel', self.left_encoder.getVelocity())
            SmartDashboard.putNumber('drive_rvel', self.right_encoder.getVelocity())
            SmartDashboard.putNumber('navX', self.navx.getAngle())

        if self.counter % 100 == 0:
            pass
            # self.display_PIDs()
            # msg = f"Positions: ({self.l_encoder.getPosition():2.2f}, {self.r_encoder.getPosition():2.2}, {self.navx.getAngle():2.2})"
            # msg = msg + f" Rates: ({self.l_encoder.getVelocity():2.2f}, {self.r_encoder.getVelocity():2.2f})  Time: {Timer.getFPGATimestamp() - self.robot.enabled_time:2.1f}"
            # SmartDashboard.putString("alert", msg)
            # SmartDashboard.putString("sparks", str(self.error_dict))
            # print(self.get_wheel_speeds(), self.l_encoder.getRate(), self.r_encoder.getRate())

    def simulationPeriodic(self) -> None:
        # do this in the functions that set the motors themselves?
        #self.dummy_motor_left.set(self.spark_neo_left_front.get())
        #self.dummy_motor_right.set(self.spark_neo_right_front.get())
        self.time = time.time()
        self.vel = self.left_encoder.getVelocity()
        dt = self.time - self.previous_time
        try:
            self.acc = (self.vel - self.previous_vel) / 0.02 # dv/ dt
        except ZeroDivisionError:
            self.acc = 0
        SmartDashboard.putNumber('drive_lvel', self.vel)
        SmartDashboard.putNumber('drive_lacc', self.acc)
        self.previous_vel = self.vel
        self.previous_time = self.time
        pose = self.get_pose()
        SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])

    def smart_motion(self, distance=0, spin=False):
        """
        Thinking of an easy way to send the motion command to the
        Can't do this when the default command is running, so have to do in a command that owns drive
        """
        multipliers = [1.0, 1.0, -1.0, -1.0] if spin else [1.0, 1.0, 1.0, 1.0]
        for controller, encoder, multiplier in zip(self.pid_controllers, self.encoders, multipliers):
            controller.setReference(encoder.getPosition() + distance*multiplier, rev.CANSparkMaxLowLevel.ControlType.kSmartMotion)

    def smart_velocity(self, velocity=0.5, spin=False):
        """
        Thinking of an easy way to send the velocity setpoint to the drivetrain
        Have to do in a command that owns drive
        """
        multipliers = [1.0, 1.0, 1.0, 1.0] if spin else [1.0, 1.0, -1.0, -1.0]
        for controller, multiplier in zip(self.pid_controllers, multipliers):
            controller.setReference(velocity * multiplier, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)


    def configure_controllers(self, pid_only=False, burn_flash=False):
        """Set the PIDs, etc for the controllers, slot 0 is position and slot 1 is velocity"""

        for i, controller in enumerate(self.pid_controllers):
            self.error_dict.update({'kP0_'+str(i):controller.setP(self.PID_dict_pos['kP'], 0)})
            self.error_dict.update({'kP1_'+str(i):controller.setP(self.PID_dict_vel['kP'], 1)})
            self.error_dict.update({'kI0_'+str(i):controller.setI(self.PID_dict_pos['kI'], 0)})
            self.error_dict.update({'kI1_'+str(i):controller.setI(self.PID_dict_vel['kI'], 1)})
            self.error_dict.update({'kD0_'+str(i):controller.setD(self.PID_dict_pos['kD'], 0)})
            self.error_dict.update({'kD_1'+str(i):controller.setD(self.PID_dict_vel['kD'], 1)})
            self.error_dict.update({'kFF_0'+str(i):controller.setFF(self.PID_dict_pos['kFF'], 0)})
            self.error_dict.update({'kFF_1'+str(i):controller.setFF(self.PID_dict_vel['kFF'], 1)})
            self.error_dict.update({'kIZ_0'+str(i):controller.setIZone(self.PID_dict_pos['kIz'], 0)})
            self.error_dict.update({'kIZ_1'+str(i):controller.setIZone(self.PID_dict_vel['kIz'], 1)})
            self.error_dict.update({'MinMax0_'+str(i):controller.setOutputRange(self.PID_dict_pos['kMinOutput'], self.PID_dict_pos['kMaxOutput'], 0)})
            self.error_dict.update({'MinMax0_'+str(i):controller.setOutputRange(self.PID_dict_vel['kMinOutput'], self.PID_dict_vel['kMaxOutput'], 1)})
            self.error_dict.update({'Accel0_'+str(i):controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 0)}) #
            self.error_dict.update({'Accel1_'+str(i):controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 1)}) #
            self.error_dict.update({'Vel0_'+str(i):controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 0)}) #
            self.error_dict.update({'Vel1_'+str(i):controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 1)}) #

        if not pid_only:
            for i, controller in enumerate(self.controllers):
                # error_dict.append(controller.restoreFactoryDefaults())
                self.error_dict.update({'OpenRamp_' + str(i): controller.setOpenLoopRampRate(self.ramp_rate)})
                self.error_dict.update({'ClosedRamp_' + str(i): controller.setClosedLoopRampRate(self.ramp_rate)})
                self.error_dict.update({'CurLimit_'+str(i):controller.setSmartCurrentLimit(self.current_limit)})
                self.error_dict.update({'VoltComp_'+str(i):controller.enableVoltageCompensation(12)})
                # some of these got moved to the controller, not the PID controller
                # self.error_dict.update({'Idle_' + str(i): controller.setIdleMode(rev.IdleMode.kBrake)})
                # defaults are 10, 20, 20 on the frame rates - trying to cut down a bit on CAN bandwidth
                #self.error_dict.update({'PeriodicStatus0_'+str(i):controller.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus0, 20)})
                #self.error_dict.update({'PeriodicStatus1_'+str(i):controller.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus1, 40)})
                #self.error_dict.update({'PeriodicStatus2_'+str(i):controller.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus2, 20)})

        if len(set(self.error_dict)) > 1:
            print('\n*Sparkmax setting*     *Response*')
            for key in sorted(self.error_dict.keys()):
                print(f'     {key:15} \t {self.error_dict[key]}', flush=True)
        else:
            print(f'\n *All SparkMax report {list(set(self.error_dict))[0]}')

        if burn_flash:
            start_time = time.time()
            for i, controller in enumerate(self.controllers):
                can_error = controller.burnFlash()
                print(f'Burn flash on controller {i}: {can_error} {int(1000*(time.time()-start_time)):2d}ms after starting')

    def set_brake_mode(self, mode):
        brake_mode = rev.CANSparkMax.IdleMode.kBrake
        if mode == 'coast':
            brake_mode = rev.CANSparkMax.IdleMode.kCoast

        for controller in self.controllers:
            controller.setIdleMode(brake_mode)
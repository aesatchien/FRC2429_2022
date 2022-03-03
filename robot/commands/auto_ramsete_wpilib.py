"""
CJH moving the wpilib ramsete example outside of robotcontainer
"""
from commands2 import RunCommand, RamseteCommand, CommandBase
from wpimath.controller import RamseteController, PIDController, SimpleMotorFeedforwardMeters
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator, Trajectory
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d
from subsystems.drivetrain import Drivetrain
import constants

class AutoRamseteWpilib(CommandBase):

    def __init__(self, container, drive: Drivetrain):
        # Create a voltage constraint to ensure we don't accelerate too fast.
        super().__init__()
        self.container = container
        self.drive = drive
        self.addRequirements(self.drive)

        autoVoltageConstraint = DifferentialDriveVoltageConstraint(
            SimpleMotorFeedforwardMeters(
                constants.ks_volts,
                constants.kv_volt_seconds_per_meter,
                constants.ka_volt_seconds_squared_per_meter,
            ),
            constants.k_drive_kinematics,
            maxVoltage=10,  # 10 volts max.
        )

        # Below will generate the trajectory using a set of programmed configurations
        # Create a configuration for the trajctory. This tells the trajectory its constraints
        # as well as its resources, such as the kinematics object.
        config = TrajectoryConfig(constants.k_max_speed_meters_per_second,
            constants.k_max_acceleration_meters_per_second_squared,)
        # Ensures that the max speed is actually obeyed.
        config.setKinematics(constants.k_drive_kinematics)
        # Apply the previously defined voltage constraint.
        config.addConstraint(autoVoltageConstraint)
        # Start at the origin facing the +x direction.
        initialPosition = Pose2d(0, 0, Rotation2d(0))
        # Here are the movements we also want to make during this command.
        # These movements should make an "S" like curve.
        movements = [Translation2d(1, 1), Translation2d(2, -1)]
        # End at this position, three meters straight ahead of us, facing forward.
        finalPosition = Pose2d(3, 0, Rotation2d(0))
        # An example trajectory to follow. All of these units are in meters.
        exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            initialPosition,
            movements,
            finalPosition,
            config,
        )

        # instead of resetting the odometer, translate the trajectory instead
        transform = Transform2d(exampleTrajectory.initialPose(), self.drive.get_pose())
        exampleTrajectory = exampleTrajectory.transformBy(transform)

        # Below creates the RAMSETE command

        ramseteCommand = RamseteCommand(
            # The trajectory to follow.
            exampleTrajectory,
            # A reference to a method that will return our position.
            self.drive.get_pose,
            # Our RAMSETE controller.
            RamseteController(constants.k_ramsete_b, constants.k_ramsete_zeta),
            # A feedforward object for the robot.
            SimpleMotorFeedforwardMeters(
                constants.ks_volts,
                constants.kv_volt_seconds_per_meter,
                constants.ka_volt_seconds_squared_per_meter,
            ),
            # Our drive kinematics.
            constants.k_drive_kinematics,
            # A reference to a method which will return a DifferentialDriveWheelSpeeds object.
            self.drive.get_wheel_speeds,
            # The turn controller for the left side of the drivetrain.
            PIDController(constants.k_kp_drive_vel, 0, 0),
            # The turn controller for the right side of the drivetrain.
            PIDController(constants.k_kp_drive_vel, 0, 0),
            # A reference to a method which will set a specified
            # voltage to each motor. The command will pass the two parameters.
            self.drive.tank_drive_volts,
            # The subsystems the command should require.
            [self.drive],
        )

        # Reset the robot's position to the starting position of the trajectory.
        # self.robotDrive.resetOdometry(self.exampleTrajectory.initialPose())

        # Return the command to schedule. The "andThen()" will halt the robot after
        # the command finishes.
        #return ramseteCommand.andThen(lambda: self.drive.tankDriveVolts(0, 0))
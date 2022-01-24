#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import wpilib
import wpilib.simulation
from wpilib.simulation import SimDeviceSim, DifferentialDrivetrainSim
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor
import wpimath.geometry as geo

import constants

from pyfrc.physics.core import PhysicsInterface


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(constants.kLeftMotor1Port)
        self.r_motor = wpilib.simulation.PWMSim(constants.kRightMotor1Port)

        # Encoders
        self.leftEncoderSim = wpilib.simulation.EncoderSim.createForChannel(
            constants.kLeftEncoderPorts[0]
        )
        self.rightEncoderSim = wpilib.simulation.EncoderSim.createForChannel(
            constants.kRightEncoderPorts[0]
        )

        self.system = LinearSystemId.identifyDrivetrainSystem(constants.kvVoltSecondsPerInch,
                                                              constants.kaVoltSecondsSquaredPerInch, 25.0, 1.6)
        self.drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self.system,
            constants.kTrackWidth,
            DCMotor.CIM(constants.kDriveTrainMotorCount),
            constants.kGearingRatio,
            constants.kWheelRadius,
        )

        #Gyros
        print(f'** EnumerateDevices: **\n{SimDeviceSim.enumerateDevices()} ')
        self.gyro = SimDeviceSim('Gyro:RomiGyro')
        print(f'** Gyro:RomiGyro allows access to: **\n{self.gyro.enumerateValues()} ')
        self.gyro_angle_z = self.gyro.getDouble('angle_z')  # RomiGyro value

        # initial position
        self.x, self.y = 2, 3
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
        self.pose = geo.Pose2d(self.x, self.y, geo.Rotation2d(0))
        initial_position_transform = geo.Transform2d(initial_pose, self.pose)
        self.physics_controller.move_robot(initial_position_transform)


    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        voltage = wpilib.RobotController.getInputVoltage()
        self.drivesim.setInputs(l_motor * voltage, -r_motor * voltage)
        self.drivesim.update(tm_diff)

        self.leftEncoderSim.setDistance(self.drivesim.getLeftPosition())
        self.leftEncoderSim.setRate(self.drivesim.getLeftVelocity())
        self.rightEncoderSim.setDistance(self.drivesim.getRightPosition())
        self.rightEncoderSim.setRate(self.drivesim.getRightVelocity())

        # Update the romi gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but the returned pose is positive counter-clockwise
        self.gyro_angle_z.set(-self.drivesim.getHeading().degrees())

        self.physics_controller.field.setRobotPose(self.drivesim.getPose())

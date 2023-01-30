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
import math

from wpilib import RobotController, SmartDashboard
from wpilib.simulation import DifferentialDrivetrainSim
import wpilib.simulation as simlib  # 2021 name for the simulation library
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor
import wpimath.geometry as geo
# from networktables import NetworkTables
import ntcore as nt

import constants

from pyfrc.physics.core import PhysicsInterface


class PhysicsEngine:
    """
    Updated 2022 0103 CJH as a template for commandsv2
    2429's sim uses the latest conventions from the robotpy commandsv2 examples, except:
    1) We use SparkMAX controllers for drivetrain, and since they do not update their sim device we
      a) use two dummy PWM motors that follow the actual controllers in simulationPeriodic()
      b) read those dummy PWM values and use them to update the SparkMAX's SimDeviceSim
    2) We add boundaries to the field so the robot stays on the screen
      a) we can do that with a simple teleport that always pushes us back in bounds
      b) we can be more involved and undo the movement that put us out of bounds (or into an obstacle)
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller  # this is mandatory

        # variables to retain throughout simulation
        self.counter = 0

        # SparkMAX does not write to its simdevice in the real robot, so use the generic wpilib PWM as a proxy
        #
        self.l_motor = simlib.PWMSim(1)
        self.r_motor = simlib.PWMSim(3)

        # Motor simulation definitions. Each correlates to a motor defined in the drivetrain subsystem.
        self.l_spark = simlib.SimDeviceSim('SPARK MAX [1]')  # SparkMAX sim device
        self.r_spark = simlib.SimDeviceSim('SPARK MAX [3]')
        self.l_spark_position = self.l_spark.getDouble('Position')  # SparkMAX encoder distance
        self.r_spark_position = self.r_spark.getDouble('Position')
        self.l_spark_velocity = self.l_spark.getDouble('Velocity')  # SparkMAX encoder rate
        self.r_spark_velocity = self.r_spark.getDouble('Velocity')
        self.l_spark_output = self.l_spark.getDouble('Applied Output')  # SparkMAX controller output
        self.r_spark_output = self.r_spark.getDouble('Applied Output')

        self.system = LinearSystemId.identifyDrivetrainSystem(
            constants.kv_volt_seconds_per_meter,  # The linear velocity gain in volt seconds per distance.
            constants.ka_volt_seconds_squared_per_meter,  # The linear acceleration gain, in volt seconds^2 per distance.
            1.5,  # The angular velocity gain, in volt seconds per angle.
            0.3,  # The angular acceleration gain, in volt seconds^2 per angle.
        )

        # The simulation model of the drivetrain.
        self.drivesim = DifferentialDrivetrainSim(
            # The state-space model for a drivetrain.
            self.system,
            # The robot's trackwidth, which is the distance between the wheels on the left side
            # and those on the right side. The units is meters.
            constants.k_track_width_meters,
            # Four NEO drivetrain setup.
            DCMotor.NEO(constants.k_drivetrain_motor_count),
            # One to one output gearing.
            constants.k_gear_ratio,
            # The radius of the drivetrain wheels in meters.
            (constants.k_wheel_diameter_m / 2),
        )

        # NavX (SPI interface) - no idea why the "4" is there, seems to be the default name generated by the navx code
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")

        # giving ourselves boundaries - ToDo: can we ask the field for this?
        self.x_limit, self.y_limit = 18.29 - 0.25, 9.14 - 0.25  # standard competition field
        self.sim_padding = 0.0  # how much to allow the robot to go out of bounds
        self.edge_bounce = 0.0  # how much to force the robot to go back by when it hits the boundary

        # initial position
        self.x, self.y = constants.k_start_x, constants.k_start_y
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d())
        self.pose = geo.Pose2d(self.x, self.y, geo.Rotation2d().fromDegrees(constants.k_start_heading))
        initial_position_transform = geo.Transform2d(initial_pose, self.pose)
        self.drivesim.setState([self.x, self.y, self.pose.rotation().radians(), 0, 0, 0, 0])
        #self.physics_controller.move_robot(initial_position_transform)
        self.previous_pose = self.drivesim.getPose()

        key = 'green'
        self.ballcam_table = nt.NetworkTableInstance.getDefault().getTable('BallCam')
        self.targets_entry = self.ballcam_table.getEntry(f"/{key}/targets")
        self.distance_entry =  self.ballcam_table.getEntry(f"/{key}/distance")
        self.rotation_entry =  self.ballcam_table.getEntry(f"/{key}/rotation")

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """


        # ------------------  UPDATE DRIVETRAIN  --------------------
        # Simulate the drivetrain - read the dummy PWMs that are following the SparkMAXes only in sim
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        # Update the navx gyro simulation
        # -> FRC gyros like NavX are positive clockwise, but the returned pose is positive counter-clockwise
        self.navx_yaw.set(-self.drivesim.getHeading().degrees())

        voltage = RobotController.getInputVoltage()
        self.drivesim.setInputs(l_motor * voltage, -r_motor * voltage)
        self.drivesim.update(tm_diff)

        # definitely simpler to sim a drivetrain now - can pull everything we need from wpilib's drivesim
        self.l_spark_output.set(l_motor)  # update the SparkMAX motor output so you can plot
        self.r_spark_output.set(r_motor)
        self.l_spark_position.set(self.drivesim.getLeftPosition())  # update the SparkMAX encoder output
        self.r_spark_position.set(-self.drivesim.getRightPosition())
        self.l_spark_velocity.set(self.drivesim.getLeftVelocity())  # update the SparkMAX encoder rate
        self.r_spark_velocity.set(-self.drivesim.getRightVelocity())

        self.pose = self.drivesim.getPose()
        self.x, self.y, self.rot = self.pose.translation().x, self.pose.translation().y, self.pose.rotation()

        # place robot on field - choose place, reflect or bounce
        self.bounce_robot()

        # save pose for the next iteration
        self.previous_pose = self.pose

        # ------------------  UPDATE MECHANISMS  --------------------

        # ------------------  DEBUG COMMUNICATIONS  --------------------
        self.counter += 1
        if self.counter % 5 == 0:
            SmartDashboard.putNumber('/sim/field_x', round(self.x, 2))  # ramsete reads this for new trajectories
            SmartDashboard.putNumber('/sim/field_y', round(self.y, 2))
            SmartDashboard.putNumber('/sim/field_rot', round(self.rot.degrees(), 2))
            hub_dist, hub_rot = self.distance_to_hub()
            SmartDashboard.putNumber('/sim/hub_dist', round(hub_dist, 2))
            SmartDashboard.putNumber('/sim/hub_rot', round(hub_rot, 2))
            self.targets_entry.setDouble(2)
            self.distance_entry.setDouble(hub_dist)
            self.rotation_entry.setDouble(self.pose.rotation().degrees() -hub_rot)

            #SmartDashboard.putNumber('sim/state', self.drivesim)

    def distance_to_hub(self): # example way, but VERY rigid - can't drag robot
        hub_x, hub_y  = 8.25, 4.1
        dx = self.pose.X() - hub_x
        dy = self.pose.Y() - hub_y
        distance = (dx**2 + dy**2)**0.5
        rotation = math.atan2(dy, dx) * 180/math.pi
        return distance, rotation

    # ------------------  ROBOT PLACEMENT OPTIONS --------------------
    def place_robot(self): # example way, but VERY rigid - can't drag robot
        self.physics_controller.field.setRobotPose(self.drivesim.getPose())

    def reflect_robot(self):  # slightly better, at least you are always on screen
        reflected_pose = geo.Pose2d(x=self.x % self.x_limit, y=self.y % self.y_limit, rotation=self.rot)
        self.physics_controller.field.setRobotPose(reflected_pose)

    def bounce_robot(self):
        # correct for going out of bounds - simple teleport back to the boundary for now
        movement_transform = self.pose - self.previous_pose  # how much we're intending to move
        if (self.x < -self.sim_padding or self.x > self.x_limit + self.sim_padding or
                self.y < -self.sim_padding or self.y > self.y_limit + self.sim_padding):
            self.x = constants.clamp(value=self.x, bottom=0.05 + self.edge_bounce, top=self.x_limit-self.edge_bounce -0.05)
            self.y = constants.clamp(value=self.y, bottom=0.05 + self.edge_bounce, top=self.y_limit-self.edge_bounce -0.05)
            self.pose = geo.Pose2d(self.x, self.y, self.rot)
            movement_transform = geo.Transform2d(0, 0, 0)
            # movement_transform = movement_transform.inverse()
            self.drivesim.setState([self.x, self.y, self.previous_pose.rotation().radians(),
                                   self.drivesim.getLeftVelocity(), self.drivesim.getRightVelocity(),
                                    self.drivesim.getLeftPosition(), self.drivesim.getRightPosition()])
            self.previous_pose = self.previous_pose
        else:
            pass

        #self.physics_controller.field.setRobotPose(self.pose)
        self.pose = self.physics_controller.move_robot(movement_transform)
        #self.drivesim.setPose(self.pose)



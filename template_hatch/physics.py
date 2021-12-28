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
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor
import wpimath.geometry as geo
from wpilib import SmartDashboard


import constants

from pyfrc.physics.core import PhysicsInterface

def clamp(value: float, bottom: float, top: float) -> float:
    return max(bottom, min(value, top))

def distance(pose, point):
    """ Find the distance between a pose and a point """
    return ((pose.translation().x - point[0])**2 + (pose.translation().y - point[1])**2)**0.5

class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface):

        self.physics_controller = physics_controller

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(1)
        self.r_motor = wpilib.simulation.PWMSim(2)
        self.elevator_motor = wpilib.simulation.PWMSim(5)

        self.system = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3)
        self.drivesim = wpilib.simulation.DifferentialDrivetrainSim(
            self.system,
            constants.kTrackWidth,
            DCMotor.CIM(constants.kDriveTrainMotorCount),
            constants.kGearingRatio,
            constants.kWheelRadius,
        )

        self.leftEncoderSim = wpilib.simulation.EncoderSim.createForChannel(constants.kLeftEncoderPorts[0])
        self.rightEncoderSim = wpilib.simulation.EncoderSim.createForChannel(constants.kRightEncoderPorts[0])
        self.elevator_encoder = wpilib.simulation.EncoderSim.createForChannel(4)
        self.elevator_encoder.setDistancePerPulse(1 / 2048)

        # CJH added 2012 1227
        self.sim_padding = 0.0
        self.edge_bounce = 0.0
        self.x_limit, self.y_limit = 15.47 + 0.5, 8.21 + 0.5
        # grab four obstacles from the 2020 field
        obstacles = [(225, 334-131), (359, 334-75), (412, 334-201), (277, 334-257)]
        self.obstacles = [(self.x_limit/640 * i[0], self.y_limit/334 * i[1]) for i in obstacles]
        print(self.obstacles)

        self.elevator_position = 0

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

        self.old_pose = self.drivesim.getPose()
        self.lpos, self.rpos = self.drivesim.getLeftPosition(), self.drivesim.getRightPosition()
        self.lvel, self.rvel = self.drivesim.getLeftVelocity(), self.drivesim.getRightVelocity()

        voltage = wpilib.RobotController.getInputVoltage()
        self.drivesim.setInputs(l_motor * voltage, -r_motor * voltage)

        self.drivesim.update(tm_diff)

        self.leftEncoderSim.setDistance(self.drivesim.getLeftPosition() * 39.37)
        self.leftEncoderSim.setRate(self.drivesim.getLeftVelocity() * 39.37)
        self.rightEncoderSim.setDistance(self.drivesim.getRightPosition() * 39.37)
        self.rightEncoderSim.setRate(self.drivesim.getRightVelocity() * 39.37)

        self.physics_controller.field.setRobotPose(self.drivesim.getPose())

        # CJH added 2012 1227
        pose = self.drivesim.getPose()
        self.x, self.y, self.rot = pose.translation().x, pose.translation().y, pose.rotation()

        # keep us on the simulated field - reverse the transform if we try to go out of bounds
        bad_move = False  # see if we are out of bounds or hitting a barrier
        if (pose.translation().x < -self.sim_padding or pose.translation().x > self.x_limit + self.sim_padding or
                pose.translation().y < -self.sim_padding or pose.translation().y > self.y_limit + self.sim_padding):
            bad_move = True
        if any([distance(pose, i) < 0.5 for i in self.obstacles]):
            bad_move = True


        if bad_move:  # keep us in-bounds
            x = clamp(value=self.x, bottom= self.edge_bounce, top=self.x_limit-self.edge_bounce)
            y = clamp(value=self.y, bottom=self.edge_bounce, top=self.y_limit-self.edge_bounce)
            #self.drivesim.setPose(geo.Pose2d(x, y, self.rot))

            # or undo the motion
            dx, dy = self.x - self.old_pose.X(), self.y - self.old_pose.Y()
            self.drivesim.setPose(geo.Pose2d(self.x-2*dx, self.y-2*dy, self.rot))

            self.physics_controller.field.setRobotPose(self.drivesim.getPose())
            SmartDashboard.putNumber('dx', round(dx, 2))

        SmartDashboard.putNumber('field_x', round(self.x, 2))
        SmartDashboard.putNumber('field_y', round(self.y, 2))
        SmartDashboard.putNumber('field_rot', round(self.rot.degrees(), 2))

        # physics for an elevator
        # update 'position' (use tm_diff so the rate is constant) - this is for simulating an elevator, arm etc w/ limit switches
        self.elevator_position += self.elevator_motor.getSpeed() * tm_diff * 200 - 1.0  # inserting something like gravity...
        self.elevator_position = clamp(value=self.elevator_position, bottom=0, top=100)
        self.elevator_encoder.setDistance(round(self.elevator_position, 2))
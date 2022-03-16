"""
Updated 2022 0127 CJH
This command recreates an commands2 ramsete command from the individual pieces so we can get more trajectory data

"""

from datetime import datetime
import pickle
from pathlib import Path

import commands2
from wpilib import SmartDashboard
from wpimath import controller
import wpimath.kinematics
import wpimath.geometry as geo
from networktables import NetworkTablesInstance

from subsystems.drivetrain import Drivetrain
import constants  # 2429's drive constants
import trajectory_io  # helper file for generating trajectories from our list of paths


class AutoRamsete(commands2.CommandBase):

    # constants for ramsete follower and velocity PID controllers - don't want a different copy for each command
    beta = constants.k_ramsete_b
    zeta = constants.k_ramsete_zeta
    kp_vel = constants.k_kp_drive_vel
    kd_vel = 0
    velocity = constants.k_max_speed_meters_per_second
    write_telemetry = False

    dash = True  # ToDo: decide if I ever want to hide this or to make it
    if dash is True:
        ntinst = NetworkTablesInstance.getDefault()
        ramsete_table = ntinst.getTable('Ramsete')
        ramsete_table.putNumber("ramsete_kpvel", kp_vel)
        ramsete_table.putNumber("ramsete_B", beta)
        ramsete_table.putNumber("ramsete_Z", zeta)
        ramsete_table.putBoolean("ramsete_write", write_telemetry)
        ramsete_table.putNumber("waypoint_x", 1)
        ramsete_table.putNumber("waypoint_y", 0)
        ramsete_table.putNumber("waypoint_heading", 0)
        ramsete_table.putBoolean("waypoint_reverse", False)

        #SmartDashboard.putNumber("/ramsete/ramsete_kpvel", kp_vel)
        #SmartDashboard.putNumber("/ramsete/ramsete_B", beta)
        #SmartDashboard.putNumber("/ramsete/ramsete_Z", zeta)
        #SmartDashboard.putBoolean("/ramsete/ramsete_write", write_telemetry)
        #SmartDashboard.putNumber("/ramsete/waypoint_x", 1)
        #SmartDashboard.putNumber("/ramsete/waypoint_y", 0)
        #SmartDashboard.putNumber("/ramsete/waypoint_heading", 0)
        #SmartDashboard.putBoolean("/ramsete/waypoint_reverse", False)


    def __init__(self, container, drive: Drivetrain, relative=True, source=None, trajectory=None) -> None:
        super().__init__()
        self.setName('AutoRamsete')
        self.drive = drive
        self.container = container
        self.relative = True  # used to see if we will use absolute paths or relative ones
        self.addRequirements(drive)  # commandsv2 version of requirements
        #self.withTimeout(10)

        self.previous_time = -1
        self.previous_speeds = None
        self.use_PID = True
        self.counter = 0
        self.telemetry = []
        self.source = source
        self.trajectory = trajectory

        self.feed_forward = constants.k_feed_forward
        self.kinematics = constants.k_drive_kinematics
        self.start_time = None
        self.course = None

        self.exit = False  # it is so hard to kill this command

    def initialize(self) -> None:
        self.init_time = self.container.get_enabled_time()
        self.start_time = self.container.get_enabled_time()
        self.previous_time = -1
        self.telemetry = []

        self.container.robot_drive.drive.feed()  # this initialization is taking some time now
        # update gains from dash if desired
        if self.dash is True:
            # self.kp_vel = SmartDashboard.getNumber("/ramsete/ramsete_kpvel", self.kp_vel)
            # self.beta = SmartDashboard.getNumber("/ramsete/ramsete_B", self.beta)
            # self.zeta = SmartDashboard.getNumber("/ramsete/ramsete_Z", self.zeta)
            # self.write_telemetry = SmartDashboard.getBoolean("/ramsete/ramsete_write", self.write_telemetry)
            self.kp_vel = self.ramsete_table.getNumber('ramsete_kpvel', self.kp_vel)
            self.beta = self.ramsete_table.getNumber('ramsete_B', self.beta)
            self.zeta = self.ramsete_table.getNumber('ramsete_Z', self.zeta)
            self.write_telemetry = self.ramsete_table.getBoolean('ramsete_write', self.write_telemetry)
            print(self.kp_vel, self.beta, self.zeta, self.write_telemetry)
        # create controllers
        self.follower = controller.RamseteController(self.beta, self.zeta)
        self.left_controller = controller.PIDController(self.kp_vel, 0, self.kd_vel)
        self.right_controller = controller.PIDController(self.kp_vel, 0, self.kd_vel)
        self.left_controller.reset()
        self.right_controller.reset()

        if self.source is None or self.source == 'pathweaver':
            trajectory_choice = self.container.path_chooser.getSelected()  # get path from the GUI
            self.velocity = float(self.container.velocity_chooser.getSelected())  # get the velocity from the GUI
            if 'z_' not in trajectory_choice:  # let me try a few of the other methods if the path starts with z_
                self.trajectory = trajectory_io.generate_trajectory(trajectory_choice, self.velocity, display=False, save=False)
                self.course = trajectory_choice
                if not constants.k_is_simulation:
                    self.start_pose = geo.Pose2d(self.trajectory.sample(0).pose.X(), self.trajectory.sample(0).pose.Y(),
                                                 self.container.robot_drive.get_rotation2d())
                    self.container.robot_drive.reset_odometry(self.start_pose)
                else:
                    field_x = SmartDashboard.getNumber('/sim/field_x', self.trajectory.sample(0).pose.X())
                    field_y = SmartDashboard.getNumber('/sim/field_y', self.trajectory.sample(0).pose.Y())
                    self.start_pose = geo.Pose2d(field_x, field_y, self.container.robot_drive.get_rotation2d())
        elif self.source == 'dash':  # we told it to calculate a trajectory from a point on the dashboard
            self.velocity = float(self.container.velocity_chooser.getSelected())
            start_pose = geo.Pose2d(geo.Translation2d(x=0, y=0), geo.Rotation2d(0.000000))
            end_x = self.ramsete_table.getNumber("waypoint_x", 1)
            end_y = self.ramsete_table.getNumber("waypoint_y", 0)
            heading = self.ramsete_table.getNumber("waypoint_heading", 0)
            reverse = self.ramsete_table.getBoolean("waypoint_reverse", False)

            print(end_x, end_y, heading, reverse)
            if reverse:
                end_x, end_y, heading = -end_x, -end_y, -heading
            end_pose = geo.Pose2d(geo.Translation2d(x=end_x, y=end_y), geo.Rotation2d().fromDegrees(heading))
            try:
                self.trajectory = trajectory_io.generate_trajectory_from_points(waypoints=[start_pose, end_pose],
                                                    velocity=self.velocity, reverse=reverse, display=False, save=True)
            except Exception as e:  # don't send it a trajectory it can't calculate
                print(f'Error generating trajectory: {e}')
                self.end(interrupted=True)

        elif self.source == 'trajectory':  # we sent it a trajectory when we called the function
            pass

        elif self.source == 'ball': # generate a tajectory from ball location
            (ball_detected, rotation_offset, distance) = self.container.robot_vision.getBallValues()
            if ball_detected:
                end_x = distance - 0.25  # let's not overshoot - dist is from camera, not bumper
                reverse = False
                print(f'Atttempting ball drive, distance is {end_x}')
                success, self.trajectory = trajectory_io.generate_quick_trajectory(x=abs(end_x), y=0, velocity=1.5, reverse=reverse, display=True)
                if success == 0:
                    self.end(interrupted=True)
                    self.exit = True
            else:
                success, self.trajectory = trajectory_io.generate_quick_trajectory(x=abs(0), y=0, velocity=1.5, reverse=False, display=True)
                self.end(interrupted=True)
                self.exit = True

        elif self.source == 'hub':  # generate a trajectory from hub location
            (hub_detected, rotation_offset, distance) = self.container.robot_vision.getHubValues()
            if hub_detected:
                ideal_distance = 1.3
                end_x = ideal_distance - distance  # positive if we have to move away
                reverse = end_x < 0  # we are facing away from target
                print(f'Atttempting hub drive, distance is {end_x}')
                success, self.trajectory = trajectory_io.generate_quick_trajectory(x=abs(end_x), y=0, velocity=1.5, reverse=reverse, display=True)
                if success == 0:
                    self.end(interrupted=True)
                    self.exit = True
            else:
                success, self.trajectory = trajectory_io.generate_quick_trajectory(x=abs(0), y=0, velocity=1.5, reverse=False, display=True)
                self.end(interrupted=True)
                self.exit = True
        else:
            pass  # just let it die

        if self.trajectory is not None:
            if self.relative:  # clean way to take a trajectory and shift it to new start location and orientation
                # more than one way to update the trajectory - reset the drive odometer to initial pose
                # or use the current odometry reading as the beginning of the trajectory

                transform = geo.Transform2d(self.trajectory.initialPose(), self.drive.get_pose())
                self.trajectory = self.trajectory.transformBy(transform)

            self.container.robot_drive.drive.feed()  # this initialization is taking some time now

            #self.container.robot_drive.reset_odometry(self.start_pose)
            initial_state = self.trajectory.sample(0)
            # these are all meters in 2021
            self.previous_speeds = self.kinematics.toWheelSpeeds(wpimath.kinematics.ChassisSpeeds(
                initial_state.velocity, 0, initial_state.curvature * initial_state.velocity))

            self.start_time = self.container.get_enabled_time()
            print("\n" + f"** Started {self.__class__.__name__} / {self.getName()} on {self.course} with load time {1000*(self.start_time-self.init_time):2.2f}ms"
                         f" (b={self.beta}, z={self.zeta}, kp_vel={self.kp_vel}) at {self.start_time:.1f} s **")
            print(f'Attempting to run trajectory from {self.trajectory.initialPose()} to {self.trajectory.states()[-1].pose}')
            SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time:.1f} s **")

            print('Time\tTr Vel\tTr Rot\tlspd\trspd\tram ang\tram vx\tram vy\tlffw\trffw\tlpid\trpid')
        else:
            print(f'No valid trajectory obtained.', flush=True)

    def execute(self) -> None:
        current_time = self.container.get_enabled_time() - self.start_time
        dt = current_time - self.previous_time

        if self.previous_time < 0 or dt <= 0:  # had to add this <= on dt
            self.container.robot_drive.tank_drive_volts(0, 0)
            self.previous_time = current_time
            return

        # get the robot's current field pose, current trajectory point, and feed to the ramsete controller
        pose = self.container.robot_drive.get_pose()
        sample = self.trajectory.sample(current_time)
        ramsete = self.follower.calculate(pose, sample)
        target_wheel_speeds = self.kinematics.toWheelSpeeds(ramsete)

        left_speed_setpoint = target_wheel_speeds.left
        right_speed_setpoint = target_wheel_speeds.right

        v_limit = constants.k_max_voltage # for some reason the ffwd is putting out monster values of voltage - probably an error on my part in the characterization
        if self.use_PID:
            left_feed_forward = self.feed_forward.calculate(left_speed_setpoint, (left_speed_setpoint - self.previous_speeds.left)/dt)
            left_feed_forward = v_limit if left_feed_forward > v_limit else left_feed_forward
            left_feed_forward = -v_limit if left_feed_forward < -v_limit else left_feed_forward

            right_feed_forward = self.feed_forward.calculate(right_speed_setpoint, (right_speed_setpoint - self.previous_speeds.right)/dt)
            right_feed_forward = v_limit if right_feed_forward > v_limit else right_feed_forward
            right_feed_forward = -v_limit if right_feed_forward < -v_limit else right_feed_forward

            #ws_left, ws_right = self.container.robot_drive.get_wheel_speeds().left, self.container.robot_drive.get_wheel_speeds().right
            ws_left, ws_right = self.container.robot_drive.get_rate(self.container.robot_drive.get_left_encoder()), -self.container.robot_drive.get_rate(self.container.robot_drive.get_right_encoder())
            left_output_pid = self.left_controller.calculate(ws_left, left_speed_setpoint)
            right_output_pid = self.right_controller.calculate(ws_right, right_speed_setpoint)
            # 100% sure that these signs are right - see plots.   Lots of issues here.  Need to sort out where you correct for encoder signs.
            pid_sign = 1
            left_output = pid_sign * left_output_pid + left_feed_forward
            right_output = pid_sign * right_output_pid + right_feed_forward

        else:  # ToDo - fix this to just be the feed forwards and test it
            left_output = left_speed_setpoint
            right_output = right_speed_setpoint
            # dummy values for other stuff just to test printing
            ws_left, ws_right = -1, 0
            left_feed_forward, right_feed_forward = -2, 0
            left_output_pid, right_output_pid = -3, 0

        self.container.robot_drive.tank_drive_volts(left_output, right_output)
        self.previous_speeds = target_wheel_speeds
        self.previous_time = current_time
        self.container.robot_drive.drive.feed()  # should this be in tank drive?

        if self.counter % 5 == 0:  # ten times per second update the telemetry array
            telemetry_data = {'TIME':current_time, 'RBT_X':pose.X(), 'RBT_Y':pose.Y(), 'RBT_TH':pose.rotation().radians(),
                            'RBT_VEL':self.container.robot_drive.get_average_encoder_rate(),
                            'RBT_RVEL':ws_right, 'RBT_LVEL':ws_left,
                            'TRAJ_X':sample.pose.X(), 'TRAJ_Y':sample.pose.Y(), 'TRAJ_TH':sample.pose.rotation().radians(), 'TRAJ_VEL':sample.velocity,
                            'RAM_VELX':ramsete.vx, 'RAM_LVEL_SP':left_speed_setpoint, 'RAM_RVEL_SP':right_speed_setpoint,
                            'RAM_OM':ramsete.omega, 'LFF':left_feed_forward, 'RFF':right_feed_forward, 'LPID': left_output_pid, 'RPID':right_output_pid}
            self.telemetry.append(telemetry_data)
        if self.counter % 50 == 0: # once per second send data to the console
            out_string = f'{current_time:4.2f}\t{self.trajectory.sample(current_time).velocity:4.1f}\t{self.trajectory.sample(current_time).pose.rotation().radians():4.2f}\t'
            out_string += f'{left_speed_setpoint:4.2f}\t{right_speed_setpoint:4.2f}\t{ramsete.omega:4.2f}\t{ramsete.vx:4.2f}\t{ramsete.vy:4.2f}\t'
            out_string += f'{left_feed_forward:4.2f}\t{right_feed_forward:4.2f}\t{left_output_pid:4.2f}\t{right_output_pid:4.2f}'
            print(out_string)
        self.counter += 1

    def isFinished(self) -> bool:  # ToDo: investigate the different end conditions for a ramsete process
        # for now just give it as much time as the trajectory calculated it needed
        return (self.container.get_enabled_time() - self.start_time) > self.trajectory.totalTime() or self.exit

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        self.container.robot_drive.tank_drive_volts(0, 0)  # why was this commented out?

        self.write_telemetry = SmartDashboard.getBoolean("/ramsete/ramsete_write", self.write_telemetry)
        if self.write_telemetry:
            location = Path.cwd() if constants.k_is_simulation else Path('/home/lvuser/py/')  # it's not called robot on the robot
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            file_name = timestamp + '_' + self.course + f'_kpv_{self.kp_vel:2.1f}'.replace('.',
                                                        'p') + f'_vel_{str(self.velocity).replace(".", "p")}' + '.pkl'
            pickle_file = location / 'sim' / 'data' / file_name
            with open(pickle_file.absolute(), 'wb') as fp:
                out_dict = {'TIMESTAMP': timestamp, 'DATA': self.telemetry, 'COURSE': self.course,
                            'VELOCITY': self.velocity,
                            'KP_VEL': self.kp_vel, 'KD_VEL': self.kd_vel, 'BETA': self.beta, 'ZETA': self.zeta}
                pickle.dump(out_dict, fp)
            print(f'*** Wrote ramsete command data to {file_name} ***')
        else:
            print(f'*** Skipping saving of telemetry to disk ***')
from pathlib import Path
import glob

import wpimath
import wpimath.trajectory
import wpimath.geometry as geo
from wpimath.trajectory.constraint import CentripetalAccelerationConstraint

import constants

# find all of the pathweaver paths available to the robot - use to populate a drop-down sendable chooser in the UI
def get_pathweaver_paths():  # use this to fill the drop down for file selection
    # instead of passing robot simulation, check for directory exists on the robot
    path_files = glob.glob(constants.k_path_location + '*', recursive=True)
    #print(f'** Pathweaver files: {path_files} **')
    path_names = [Path(file).name for file in path_files]
    return path_names


# used in ramsete command
def generate_trajectory(path_name:str, velocity=constants.k_max_speed_meters_per_second, display=False, save=True) -> wpimath.trajectory:
    """
    Generate a wpilib trajectory from a pathweaver path.  Accepts regular and reversed paths.

    :param path_name: name of pathweaver file to be imported
    :param velocity: Maximum robot velocity for the generated trajectory
    :param save: Option to save generated trajectory to disk as 'test.json'
    :param display: Option to print to console
    :return: generated trajectory
    """

    # pathweaver_y_offfset = 4.572  # pathweaver 2021, starts at the top of the field
    pathweaver_y_offfset = 8.218  # pathweaver 2022, starts Y at the top of the field

    p = Path(constants.k_path_location + path_name)
    if p.is_file():
        lines = []
        with open(p, "r") as f:
            for line in f:
                currentline = line.split(",")
                lines.append(currentline)
        cvector_list = [wpimath.spline.Spline5.ControlVector((float(row[0]), float(row[2]), 0), (float(row[1]) + pathweaver_y_offfset, float(row[3]), 0))
                        for ix, row in enumerate(lines[1:])]

        config = wpimath.trajectory.TrajectoryConfig(velocity, constants.k_max_acceleration_meters_per_second_squared)
        config.setKinematics(constants.k_drive_kinematics)
        config.addConstraint(constants.k_autonomous_voltage_constraint)
        config.addConstraint(CentripetalAccelerationConstraint(constants.k_max_centripetal_acceleration_meters_per_second_squared))
        # check to see if the path is to be reversed it is marked in the th column of the pathweaver file
        reverse_array = [row[5] for row in lines[1:]]
        if any(entry == 'true' for entry in reverse_array):
            config.setReversed(True)
        pw_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(cvector_list, config)
        if save:
            wpimath.trajectory.TrajectoryUtil.toPathweaverJson(pw_trajectory, 'pathweaver\\test.json')
    else:
        pw_trajectory = None  # do something else? generate an empty trajectory?
        print(f'Trajectory: {p} not found', flush=True)
    if display:
        print(pw_trajectory.states)
    return pw_trajectory

# used in ramsete command
def generate_trajectory_from_points(waypoints=None, velocity=constants.k_max_speed_meters_per_second, midpoint=False, reverse=False, display=False, save=True) -> wpimath.trajectory:
    """
    Generate a wpilib trajectory from a list of points.  Accepts regular and reversed paths.
    :param velocity: Maximum robot velocity for the generated trajectory
    :param save: Option to save generated trajectory to disk as 'test.json'
    :param display: Option to print to console
    :return: generated trajectory
    """
    config = wpimath.trajectory.TrajectoryConfig(velocity, constants.k_max_acceleration_meters_per_second_squared)
    config.setKinematics(constants.k_drive_kinematics)
    config.addConstraint(constants.k_autonomous_voltage_constraint)
    config.addConstraint(CentripetalAccelerationConstraint(constants.k_max_centripetal_acceleration_meters_per_second_squared))
    # check to see if the 'reversed' parameter was passed to us
    if reverse:
        config.setReversed(True)
    if midpoint:
        midpoint_pose = geo.Pose2d(waypoints[-1].X()/2, waypoints[-1].Y()/2, geo.Rotation2d(waypoints[-1].rotation().radians()/2))
        waypoints = [waypoints[0], midpoint_pose, waypoints[-1]]
    point_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(waypoints=waypoints, config=config)
    if save:
        pass
        #wpimath.trajectory.TrajectoryUtil.toPathweaverJson(point_trajectory, 'pathweaver\\test.json')
    if display:
        print(point_trajectory.states)

    return point_trajectory

# used in ramsete command
def generate_quick_trajectory(x=1, y=0, heading=0, velocity=constants.k_max_speed_meters_per_second, reverse=False, display=False) -> wpimath.trajectory:
    return_code = 1  # success
    config = wpimath.trajectory.TrajectoryConfig(3, 3) # constants.k_max_acceleration_meters_per_second_squared)
    config.setKinematics(constants.k_drive_kinematics)
    config.addConstraint(constants.k_autonomous_voltage_constraint)
    config.addConstraint(CentripetalAccelerationConstraint(constants.k_max_centripetal_acceleration_meters_per_second_squared))

    start_pose = geo.Pose2d(geo.Translation2d(x=0, y=0), geo.Rotation2d(0.000000))
    end_pose = geo.Pose2d(geo.Translation2d(x=x, y=y), geo.Rotation2d.fromDegrees(heading))
    # check to see if the 'reversed' parameter was passed to us
    if reverse:
        config.setReversed(True)
        end_pose = geo.Pose2d(geo.Translation2d(x=-x, y=-y), geo.Rotation2d.fromDegrees(-heading))

    try:
        quick_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(waypoints=[start_pose, end_pose], config=config)

    except RuntimeError as e:
        return_code = 0  # failure
        print(f'FAILED to generate trajectory with x={x}: error {e}')
        end_pose = geo.Pose2d(geo.Translation2d(x=1, y=y), geo.Rotation2d.fromDegrees(heading))
        quick_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(waypoints=[start_pose, end_pose], config=config)

    if display:
        print(x, y, heading, velocity, reverse, display, start_pose, end_pose, quick_trajectory.totalTime())
        print(f'Quick trajectory with {len(quick_trajectory.states())} states:  ', end='\n')
        _ = [print(state) for state in quick_trajectory.states()]

    return return_code, quick_trajectory

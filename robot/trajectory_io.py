from pathlib import Path
import glob

import wpimath
import wpimath.trajectory
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
def generate_trajectory(path_name:str, velocity=constants.k_max_speed_meters_per_second, save=False) -> wpimath.trajectory:
    """
    Generate a wpilib trajectory from a pathweaver path.  Accepts regular and reversed paths.

    :param path_name: name of pathweaver file to be imported
    :param velocity: Maximum robot velocity for the generated trajectory
    :param save: Option to save generated trajectory to disk as 'test.json'
    :return: generated trajectory

    """

    pathweaver_y_offfset = 4.572
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
    return pw_trajectory

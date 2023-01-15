# need a file to communicate to the robot and the sim (sim is not aware of the robot object)
import math
from pathlib import Path
import glob

from wpimath import controller

import wpimath
import wpimath.kinematics
import wpimath.geometry as geo
import wpimath.trajectory
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint, CentripetalAccelerationConstraint

# drivetrain constants
k_wheel_diameter_in = 4  # wheel diameter in inches
k_wheel_diameter_m = 4 * 0.0254  # wheel diameter in meters
k_robot_length = 30 * 0.0254
k_robot_width = 24 * 0.0254
k_robot_wheelbase = 18 * 0.5 * 0.0254

# get these from robot characterization tools - using simulated values for now
# ToDo: characterize this on the real robot
# Note - CJH tried this with a homebrew approach with sim + linear fit and got ks, kv = 1.14,2.15.
# frc-characterization got 1.2, 1.82, 1.19, track 0.399 w/ R^2=1 when multiplying by 12V
# frc-characterization got 1.39, 1.79, 1.16, track 0.41 w/ R^2=1 when just using tank drive as specified in the tool

# bunch of historical values
sim_values_8 = [1.39, 1.79, 1.16, 0.41]  # ks, kv, ka, track  for 8" wheels and 9.52 gear ratio
sim_values_6 = [1.42, 0.811, 2.54, 0.40]  # ks, kv, ka, track  for 6" wheels and 9.52 gear ratio
sim_values_4 = [1.39, 1.56, 0.355, 0.40]  # ks, kv, ka, track  for 4" wheels and 9.52 gear ratio
real_values_8in = [0.41, 0.779, 0.235, 1.13]  # best estimate from practice robot, early February
real_values_6in = [0.324, 1.15, 0.0959, 1.13]  # best estimate from practice robot, early February
real_values_4in_wcd_practice = [0.381, 1.55, 0.279, 0.71]  # best estimate from practice robot, early February
real_values_4in_wcd_comp = [0.446, 1.55, 0.328, 0.73]  # best estimate from competition robot, early April
real_values_4in_wcd_comp = [0.446, 1.55, 0.400, 0.73]  # best estimate from competition robot, early April

robot_characterization = {'KS':0.446, 'KV':1.55, 'KA':0.40, 'TRACKWIDTH':0.73}

# ToDo: change this whole file to a class file
ks_volts, kv_volt_seconds_per_meter = 0, 0
ka_volt_seconds_squared_per_meter, k_track_width_meters = 0, 0

# Now we only have one set of constants - the robot needs to match the sim.  But the tank model does not quite deliver, so fudge things to match
ks_volts = 0.6* robot_characterization['KS']  # frc-characterization consistently measures a larger ks than tankmodel uses, so correct here with factor of 0.67
kv_volt_seconds_per_meter = 1.15* robot_characterization['KV']  # also a slight correction so it characterizes the same as the drivien robot
ka_volt_seconds_squared_per_meter = 1.15* robot_characterization['KA']  #
ks_volts = robot_characterization['KS']  # frc-characterization consistently measures a larger ks than tankmodel uses, so correct here with factor of 0.67
kv_volt_seconds_per_meter = robot_characterization['KV']  # also a slight correction so it characterizes the same as the drivien robot
ka_volt_seconds_squared_per_meter = robot_characterization['KA']  #
k_track_width_meters = robot_characterization['TRACKWIDTH']  # best measured from wheel to wheel - physically 24" = 0.61m but characterized as 0.71 (28")
k_gear_ratio = 4.17

# Configure encoders and controllers
# should be wheel_diameter * pi / gear_ratio - and for the old double reduction gear box the gear ratio was 4.17:1.
# With the shifter (low gear) I think it was a 12.26.  Then new 2020 WCD gearbox is 9.52, and the tuffbox is 12.75
k_sparkmax_conversion_factor_inches = k_wheel_diameter_in * math.pi / k_gear_ratio
k_sparkmax_conversion_factor_meters = k_wheel_diameter_m * math.pi / k_gear_ratio

# pretend encoders for simulation
k_encoder_CPR = 1024 # encoder counts per revolution
# encoder_distance_per_pulse_m = wheel_diameter_m * math.pi / (encoder_CPR * gear_ratio)
k_encoder_distance_per_pulse_m = k_wheel_diameter_m * math.pi / (k_encoder_CPR)


# need to implement this somehow, probably once we have a class set up
def set_robot_parameters(robot_real=False):
    global ks_volts, kv_volt_seconds_per_meter, ka_volt_seconds_squared_per_meter, k_track_width_meters
    if robot_real:
        ks_volts = 0.41  #
        kv_volt_seconds_per_meter = 0.779  #
        ka_volt_seconds_squared_per_meter = 0.235  # 0.0  #
        k_track_width_meters = 1.13  #
    else:
        ks_volts = 1.39 # determined as the minimum to start the robot moving
        kv_volt_seconds_per_meter = 1.79   # determined as 1/slope of the vel vs volts equation
        ka_volt_seconds_squared_per_meter = 1.16  # not sure if we have one in our sim or how to calculate it
        # set up the wpilib kinematics model
        k_track_width_meters = 0.41  # 0.69 was from the model, 0.396 was from the characterization

drive_kinematics = wpimath.kinematics.DifferentialDriveKinematics(k_track_width_meters)
kp_drive_vel = 0.2  # this is kp for the PID on each side on top of the feed forward (really sensitive - still needs work)


# constants for autonomous THIS CONTROLS THE SPEEDS OF THE TRAJECTORIES GENERATED BY THIS PROGRAM
k_max_speed_meters_per_second = 2.0  # 0.75 to 1.25 is pretty reasonable for the sim
k_max_acceleration_meters_per_second_squared = 2.75 # can we accelerate this fast?
k_max_centripetal_acceleration_meters_per_second_squared = 2.75  # force slower turns?

# Reasonable baseline values for a RAMSETE follower in units of meters and seconds
ramsete_B = 2  # default 2.  like a proportional, higher is more aggressive
ramsete_Zeta = 0.9  #  default 0.7.  like a damping term, needs to be between 0 and 1


# --------------  DRIVETRAIN OBJECTS FOR TRAJECTORY TRACKING  -------------

# Create a voltage constraint to ensure we don't accelerate too fast
k_max_voltage = 8
feed_forward = controller.SimpleMotorFeedforwardMeters(ks_volts, kv_volt_seconds_per_meter, ka_volt_seconds_squared_per_meter)
autonomous_voltage_constraint = DifferentialDriveVoltageConstraint(feed_forward, drive_kinematics, k_max_voltage)

# Create config for trajectories
config = wpimath.trajectory.TrajectoryConfig(k_max_speed_meters_per_second, k_max_acceleration_meters_per_second_squared)
config.setKinematics(drive_kinematics)
config.addConstraint(autonomous_voltage_constraint)

# need to remake this on the fly when we make trajectories here
def make_config(velocity=k_max_speed_meters_per_second):
    temp_config = wpimath.trajectory.TrajectoryConfig(velocity, k_max_acceleration_meters_per_second_squared)
    temp_config.setKinematics(drive_kinematics)
    temp_config.addConstraint(autonomous_voltage_constraint)
    temp_config.addConstraint(CentripetalAccelerationConstraint(k_max_centripetal_acceleration_meters_per_second_squared))
    return temp_config


# --------------  SAMPLE TRAJECTORIES  -------------
# example trajectory to test
def get_test_trajectory(velocity=k_max_speed_meters_per_second):
    start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
    end_pose = geo.Pose2d(4, 0, geo.Rotation2d(0))
    midpoints = [geo.Translation2d(1.5, 0.5), geo.Translation2d(2.5, -0.5)]
    test_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(start_pose, midpoints, end_pose, make_config(velocity))
    return test_trajectory

# minimum slalom test - nice thing about having the points means you can change the speeds for the
# trajectory config and then you can go faster and faster.  but it's better to use the pathweaver once you have the speeds you want
def get_point_trajectory(velocity=k_max_speed_meters_per_second):
    start_pose = geo.Pose2d(0, 0, geo.Rotation2d(0))
    end_pose = geo.Pose2d(0, 1.76, geo.Rotation2d(0))
    midpoints = [geo.Translation2d(0.32, 0.01), geo.Translation2d(1.25, 0.47), geo.Translation2d(2.16, 1.71),
                    geo.Translation2d(4.74, 1.91), geo.Translation2d(5.74, 1.06), geo.Translation2d(6.06, 0.31),
                    geo.Translation2d(7.41, 0.21), geo.Translation2d(7.52, 1.50), geo.Translation2d(6.97, 1.82),
                    geo.Translation2d(6.10, 1.48), geo.Translation2d(5.87, 0.84), geo.Translation2d(5.30, 0.17),
                    geo.Translation2d(3.77, -0.05), geo.Translation2d(1.81, 0.33), geo.Translation2d(1.12, 1.36),
                    geo.Translation2d(0.22, 1.69), geo.Translation2d(-0.15, 1.76)]
    slalom_point_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(start_pose, midpoints, end_pose, make_config(velocity))
    return slalom_point_trajectory

# just waypoints of poses - try a simple loop
def get_loop_trajectory(velocity=k_max_speed_meters_per_second):
    pose_points = [geo.Pose2d(0.09, 0.02, geo.Rotation2d(0.00)),
                    geo.Pose2d(6.17, 0.16, geo.Rotation2d(0.00)), geo.Pose2d(7.62, 0.87, geo.Rotation2d(1.60)),
                    geo.Pose2d(6.06, 1.75, geo.Rotation2d(3.14)), geo.Pose2d(0.21, 1.70, geo.Rotation2d(3.14))]
    loop_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(pose_points, make_config(velocity))
    return loop_trajectory

#lots of poses from above
def get_pose_trajectory(velocity=k_max_speed_meters_per_second):
    pose_points = [geo.Pose2d(0.32, 0.01, geo.Rotation2d(0.00)),
                        geo.Pose2d(1.25, 0.47, geo.Rotation2d(1.04)), geo.Pose2d(2.16, 1.71, geo.Rotation2d(0.69)),
                        geo.Pose2d(4.74, 1.91, geo.Rotation2d(0.00)), geo.Pose2d(5.74, 1.06, geo.Rotation2d(-1.21)),
                        geo.Pose2d(6.06, 0.31, geo.Rotation2d(0.00)), geo.Pose2d(7.41, 0.21, geo.Rotation2d(0.64)),
                        geo.Pose2d(7.52, 1.50, geo.Rotation2d(2.30)), geo.Pose2d(6.97, 1.82, geo.Rotation2d(3.12)),
                        geo.Pose2d(6.10, 1.48, geo.Rotation2d(-2.05)), geo.Pose2d(5.87, 0.84, geo.Rotation2d(-2.04)),
                        geo.Pose2d(5.30, 0.17, geo.Rotation2d(-2.55)), geo.Pose2d(3.77, -0.05, geo.Rotation2d(3.12)),
                        geo.Pose2d(1.81, 0.33, geo.Rotation2d(2.38)), geo.Pose2d(1.12, 1.36, geo.Rotation2d(2.30)),
                        geo.Pose2d(0.22, 1.69, geo.Rotation2d(-3.14)), geo.Pose2d(-0.15, 1.76, geo.Rotation2d(3.14))]
    pose_trajectory = wpimath.trajectory.TrajectoryGenerator.generateTrajectory(pose_points, make_config(velocity))
    return pose_trajectory

# alternately, import a pathweaver json
# https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/pathweaver/integrating-robot-program.html#importing-a-pathweaver-json

course = 'slalom'

@DeprecationWarning
def get_pathweaver_trajectory_old(course):  # ToDo: just read the directories and get everything with a glob - simpler that way
    """ Load different trajectories based on separate velocity directories for pathweaver output"""
    trajectory_json = course[:-5] + '.wpilib.json'  # my naming convention marks the last four characters to mark the velocity
    pathweaver_dir = 'pathweaver'
    if '75' in course:
        trajectory_dir = 'vel_0p75'
    elif '25' in course:
        trajectory_dir = 'vel_1p25'
    elif '80' in course:
        trajectory_dir = 'vel_1p80'
    try:
        trajectory_path = Path.cwd() / pathweaver_dir / trajectory_dir / 'output' / trajectory_json
        pathweaver_trajectory = wpimath.trajectory.TrajectoryUtil.fromPathweaverJson(str(trajectory_path))
        print(f"*** Successfully loaded: {trajectory_path} ***")
    except Exception as ex:
        print(f"*** Unable to open trajectory: {trajectory_path} ***")
        pathweaver_trajectory = None
    return pathweaver_trajectory

def get_pathweaver_trajectory(course):
    """ Load different trajectories based on separate velocity directories for pathweaver output"""
    trajectory_json = course[:-9] + '.wpilib.json'  # my naming convention marks the last 9 chars for the directory
    pathweaver_dir = 'pathweaver'
    trajectory_dir = course[-8:]  # i made the last 8 digits the directory name
    try:
        trajectory_path = Path.cwd() / pathweaver_dir / trajectory_dir / 'output' / trajectory_json
        pathweaver_trajectory = wpimath.trajectory.TrajectoryUtil.fromPathweaverJson(str(trajectory_path))
        print(f"*** Successfully loaded: {trajectory_path} ***")
    except Exception as ex:
        print(f"*** Unable to open trajectory: {trajectory_path} ***")
        pathweaver_trajectory = None
    return pathweaver_trajectory

@DeprecationWarning
def get_pathweaver_files():
    path_files = glob.glob('../robot/pathweaver/vel*/*/*pw*', recursive=True)
    file_names = [Path(file).name[:-12] + '_' + Path(file).parent.parent.name for file in path_files]
    return file_names

def get_pathweaver_paths(simulation=True):  # use this to fill the drop down for file selection
    location = 'pathweaver/paths/*' if simulation else '/home/lvuser/py/pathweaver/paths/*'
    path_files = glob.glob(location, recursive=True)
    #print(f'** Pathweaver files: {path_files} **')
    path_names = [Path(file).name for file in path_files]
    return path_names


def generate_trajectory(path_name:str, velocity=k_max_speed_meters_per_second, simulation=True, save=False) -> object:
    """
    Generate a wpilib trajectory from a pathweaver path.  Accepts regular and reversed paths.

    :param path_name: name of pathweaver file to be imported
    :param velocity: Maximum robot velocity for the generated trajectory
    :param save: Option to save generated trajectory to disk as 'test.json'
    :return: generated trajectory

    """
    location = 'pathweaver/paths/' if simulation else '/home/lvuser/py/pathweaver/paths/'  # it's not called robot on the robot
    pathweaver_y_offfset = 4.572
    p = Path(location + path_name)
    if p.is_file():

        # pandas approach - best not to put this on the robot
        #df_points = pd.read_csv(p, sep=',', header='infer')  # ToDo: do this without pandas
        #cvector_list = [wpimath.spline.Spline5.ControlVector((row['X'], row['Tangent X'], 0),(row['Y'] + pathweaver_y_offfset, row['Tangent Y'], 0)) for ix, row in df_points.iterrows()]

        lines = []
        with open(p, "r") as f:
            for line in f:
                currentline = line.split(",")
                lines.append(currentline)
        cvector_list = [wpimath.spline.Spline5.ControlVector((float(row[0]), float(row[2]), 0), (float(row[1]) + pathweaver_y_offfset, float(row[3]), 0))
                        for ix, row in enumerate(lines[1:])]

        config = wpimath.trajectory.TrajectoryConfig(velocity, k_max_acceleration_meters_per_second_squared)
        config.setKinematics(drive_kinematics)
        config.addConstraint(autonomous_voltage_constraint)
        config.addConstraint(CentripetalAccelerationConstraint(k_max_centripetal_acceleration_meters_per_second_squared))
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



# ----------------------  FUN WITH SIMULATIONS - list the obstructions for different courses ------------
spacing = 2.5 * .3084
slalom_points = [(1,2), (2,2), (4,2), (5,2), (6,2), (7,2), (8,2), (10,2), (1,4), (2,4)]
slalom_points = [(spacing*i[0], spacing*i[1]) for i in slalom_points]
barrel_points = [(1,2), (2,2), (5, 2), (10,2), (1,4), (2,4), (8,4)]
barrel_points = [(spacing*i[0], spacing*i[1]) for i in barrel_points]
bounce_points = [(3,1), (1,2), (2,2), (3,2), (5,2), (7,2), (8,2), (8,2), (10,2), (11,2),
                 (1,4), (2,4), (3,4), (5,4), (7,4), (8,4), (8,4), (10,4), (11,4)]
bounce_points = [(spacing*i[0], spacing*i[1]) for i in bounce_points]

def distance(pose, point):
    """ Find the distance between a pose and a point """
    return ((pose.translation().x - point[0])**2 + (pose.translation().y - point[1])**2)**0.5
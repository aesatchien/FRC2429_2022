"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""
from pathlib import Path
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
import math
import wpilib.controller

# --------------  OI  ---------------
# ID for the driver's joystick (template)
k_driver_controller_port = 0
k_controller_thrust_axis = 1
k_controller_twist_axis = 4

# --------------  DRIVETRAIN  ---------------
# The CAN IDs for the drivetrain SparkMAX motor controllers
k_left_motor1_port = 1
k_left_motor2_port = 2
k_right_motor1_port = 3
k_right_motor2_port = 4

# ToDo: figure out which need to be reversed - these are not implemented yet
k_left_encoder_reversed = False
k_right_encoder_reversed = True

# drivetrain constants
k_wheel_diameter_in = 4  # wheel diameter in inches
k_wheel_diameter_m = 4 * 0.0254  # wheel diameter in meters
k_robot_length = 30 * 0.0254
k_track_width_meters = 24 * 0.0254
k_robot_wheelbase = 18 * 0.5 * 0.0254

# In meters, distance between wheels on each side of robot.
k_drive_kinematics = DifferentialDriveKinematics(k_track_width_meters)

# Encoder counts per revolution/rotation.
# k_encoder_c_p_r = 1024
# k_wheel_diameter_meters = 0.15

# Configure encoders and controllers
# should be wheel_diameter * pi / gear_ratio - and for the old double reduction gear box the gear ratio was 4.17:1.
# With the shifter (low gear) I think it was a 12.26.  Then new 2020 WCD gearbox is 9.52, and the tuffbox is 12.75
# Note that the SparkMAX counts revolutions, so we don't need counts per revolution (CPR = 1)
k_gear_ratio = 4.17
# k_sparkmax_conversion_factor_inches = k_wheel_diameter_in * math.pi / k_gear_ratio
k_sparkmax_conversion_factor_meters = k_wheel_diameter_m * math.pi / k_gear_ratio  # used in drivetrain

# --------------  SIMULATION  ---------------
# NOTE: Please do NOT use these values on your robot. Rather, characterize your
# drivetrain using the FRC Characterization tool. These are for demo purposes
# only!
robot_characterization = {'ks':0.446, 'kv':1.55, 'ka':0.40, 'track_width':0.73}
ks_volts = robot_characterization['ks']  # so far this is only used in the Ramsete command, but in 2021 we used it in tank model as well
kv_volt_seconds_per_meter = robot_characterization['kv']  # used in physics.py LinearSystemId and Ramsete
ka_volt_seconds_squared_per_meter = robot_characterization['ka']  # used in physics.py LinearSystemId and Ramsete

# The max velocity and acceleration for our autonomous trajectories
k_max_speed_meters_per_second = 3.0
k_max_acceleration_meters_per_second_squared = 2.75
k_max_centripetal_acceleration_meters_per_second_squared = 2.75
k_max_voltage = 6

k_feed_forward = wpilib.controller.SimpleMotorFeedforwardMeters(ks_volts, kv_volt_seconds_per_meter, ka_volt_seconds_squared_per_meter)
k_autonomous_voltage_constraint = DifferentialDriveVoltageConstraint(k_feed_forward, k_drive_kinematics, k_max_voltage)

# Baseline values for a RAMSETE follower in units of meters
# and seconds. These are recommended, but may be changes if wished.
k_ramsete_b = 2
k_ramsete_zeta = 0.9
# The P gain for our turn controllers.
k_kp_drive_vel = 0 # 0.2  # used in Ramsete command

# The number of motors on the robot.
k_drivetrain_motor_count = 4


def clamp(value: float, bottom: float, top: float) -> float:
    return max(bottom, min(value, top))

# container does not seem to have access to isReal()/isSimulation(), so solve it by checking directory structure
# lots of ways to do this, but sys.platform may return linux on both robot and development machines
robot_dir = '/home/lvuser/py/pathweaver/paths/'
development_dir = 'pathweaver/paths/'
if Path(robot_dir).is_dir():
    k_path_location = robot_dir
    k_is_simulation = False
else:
    k_path_location = development_dir
    k_is_simulation = True

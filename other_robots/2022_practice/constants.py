"""
A place for the constant values in the code that may be used in more than one place. 
This offers a convenient resources to teams who need to make both quick and universal
changes.
"""
from pathlib import Path
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.trajectory.constraint import DifferentialDriveVoltageConstraint
import math
import wpimath.controller

# --------------  LED  --------------
k_led_strip_port = 4 # PWM
k_led_count = 50

# --------------  OI  ---------------
# ID for the driver's joystick (template)
k_driver_controller_port = 0
k_co_driver_controller_port = 1
k_controller_thrust_axis = 1
k_controller_twist_axis = 4
# controller axis scales
k_thrust_scale = 1.4
k_twist_scale = .55

k_max_thrust_velocity = 6  # m/s 2.75
k_max_twist_velocity = 3 # Bradley 1.25
# k_max_twist_velocity = 0.75  # Aiden

# --------------  AUTONOMOUS  ---------------
k_path_velocity = 2.25
k_shooter_speed = 2700

# --------------  DRIVETRAIN  ---------------
# The CAN IDs for the drivetrain SparkMAX motor controllers
k_left_motor1_port = 1
k_left_motor2_port = 2
k_right_motor1_port = 3
k_right_motor2_port = 4
k_shifter_pneumatics_port = 3 # PCM

PID_dict_pos = {'kP': 0.002, 'kI': 0, 'kD': 0.002, 'kIz': 0, 'kFF': 0.008, 'kArbFF':0, 'kMaxOutput': 0.99, 'kMinOutput': -0.99}
PID_dict_vel = {'kP': 0.05 , 'kI': 0.0005, 'kD': 0.00, 'kIz': 0.2, 'kFF': 0.17, 'kArbFF':0, 'kMaxOutput': 0.99, 'kMinOutput': -0.99}
smartmotion_maxvel = 1000  # rpm
smartmotion_maxacc = 30 # rpm/s?
current_limit = 60
ramp_rate = 0.0


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

# Configure encoders and controllers
# should be wheel_diameter * pi / gear_ratio - and for the old double reduction gear box the gear ratio was 4.17:1.
# With the shifter (low gear) I think it was a 12.26.  Then new 2020 WCD gearbox is 9.52, and the tuffbox is 12.75
# Note that the SparkMAX counts revolutions, so we don't need counts per revolution (CPR = 1)
#k_gear_ratio = 3.7  # 4.17 # high gear 2022
k_gear_ratio = 5.39  # 4.17 # high gear 2022
# k_sparkmax_conversion_factor_inches = k_wheel_diameter_in * math.pi / k_gear_ratio
k_sparkmax_conversion_factor_meters = k_wheel_diameter_m * math.pi / k_gear_ratio  # used in drivetrain

# --------------  SIMULATION  ---------------

# k_start_x = 7.7
# k_start_y = 2.1
# k_start_heading = -90 - 12

k_start_x = 7.647
k_start_y = 1.935
k_start_heading = -90  # looking at the drawing originally tried -109

# robot_characterization = {'ks':0.446, 'kv':1.55, 'ka':0.40, 'track_width':0.73}  # 2021 bot characterization
robot_characterization = {'ks':0.291, 'kv':1.63, 'ka':0.293, 'track_width':0.89}  # 2022 climberbot

ks_volts = robot_characterization['ks']  # so far this is only used in the Ramsete command, but in 2021 we used it in tank model as well
kv_volt_seconds_per_meter = robot_characterization['kv']  # used in physics.py LinearSystemId and Ramsete
ka_volt_seconds_squared_per_meter = robot_characterization['ka']  # used in physics.py LinearSystemId and Ramsete

# The max velocity and acceleration for our autonomous trajectories
k_max_speed_meters_per_second = 3.0
k_max_acceleration_meters_per_second_squared = 3  # was 2.75 for 2021 paths
k_max_centripetal_acceleration_meters_per_second_squared = 2.75
k_max_voltage = 6

k_feed_forward = wpimath.controller.SimpleMotorFeedforwardMeters(ks_volts, kv_volt_seconds_per_meter, ka_volt_seconds_squared_per_meter)
k_autonomous_voltage_constraint = DifferentialDriveVoltageConstraint(k_feed_forward, k_drive_kinematics, k_max_voltage)

# Baseline values for a RAMSETE follower in units of meters
# and seconds. These are recommended, but may be changes if wished.
k_ramsete_b = 2
k_ramsete_zeta = 0.9
# The P gain for our turn controllers.
k_kp_drive_vel = 0.15 # 0.2  # used in Ramsete command

# The number of motors on the robot.
k_drivetrain_motor_count = 4


# --------------  HELPER FUNCTIONS  ---------------
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

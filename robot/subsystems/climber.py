import time
from commands2 import SubsystemBase
from wpilib import SmartDashboard
from wpimath.filter import MedianFilter
import rev

from networktables import NetworkTablesInstance, NetworkTables

import constants


class Climber(SubsystemBase):
    def __init__(self):
        super().__init__()
        self.setName('Climber')
        self.counter = 0

        self.current_filter = MedianFilter(5)

        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        self.climber_left_neo = rev.CANSparkMax(constants.k_climber_left_port, motor_type)
        self.climber_right_neo = rev.CANSparkMax(constants.k_climber_right_port, motor_type)

        self.climber_left_encoder = self.climber_left_neo.getEncoder()
        self.climber_right_encoder = self.climber_right_neo.getEncoder()  # not necessary if follower

        conversion_factor = 360 * 15 / (75 * 48)
        self.climber_left_encoder.setPositionConversionFactor(conversion_factor)  # 75 Rev GB * 48 big sprocket / 15 little sprocket
        self.climber_left_encoder.setVelocityConversionFactor(conversion_factor / 60)
        self.start_angle = 0  # we start at 54 degrees from the vertical
        self.climber_left_encoder.setPosition(self.start_angle)  # may need to have to worry about the sign here

        self.climber_left_controller = self.climber_left_neo.getPIDController()
        self.climber_right_controller = self.climber_right_neo.getPIDController()  # not necessary if follower

        self.climber_right_neo.follow(self.climber_left_neo, invert=True)

        self.climber_enable = False
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

        self.error_dict = {}
        self.PID_dict_pos = constants.PID_dict_pos_climber
        self.PID_dict_vel = constants.PID_dict_vel_climber
        self.smartmotion_maxvel = constants.smartmotion_maxvel
        self.smartmotion_maxacc = constants.smartmotion_maxacc

        # self.climber_left_controller.setSmartMotionAccelStrategy()

        # add encoder later
        #ntinst = NetworkTablesInstance.getDefault()
        #self.climber_table = ntinst.getTable('Climber')
        #self.climber_voltage = self.climber_table.getEntry('voltage')
        #self.climber_current = self.climber_table.getEntry('current')

    def set_voltage(self, voltage):
        self.climber_left_controller.setReference(voltage, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.climber_enable = True
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def stop_motor(self):
        self.climber_left_controller.setReference(0, rev.CANSparkMaxLowLevel.ControlType.kVoltage)
        self.climber_enable = False
        SmartDashboard.putBoolean('climber_state', self.climber_enable)

    def smart_motion(self, angle=0, relative=False):
        # send climber to a new angle
        if relative:  # move from current angle
            self.climber_left_controller.setReference(self.climber_left_encoder.getPosition() + angle, rev.CANSparkMaxLowLevel.ControlType.kSmartMotion, 0)
        else:  # set and absolute angle
            self.climber_left_controller.setReference(angle, rev.CANSparkMaxLowLevel.ControlType.kSmartMotion, 0)

    def smart_velocity(self, velocity=0):
        self.climber_left_controller.setReference(velocity, rev.CANSparkMaxLowLevel.ControlType.kSmartVelocity, 1)

    def get_angle(self):
        return self.climber_left_encoder.getPosition()

    def reset_angle(self):
        self.climber_left_encoder.setPosition(0)

    def configure_controllers(self, pid_only=False, burn_flash=False):
        """Set the PIDs, etc for the controllers, slot 0 is position and slot 1 is velocity"""

        i = 0
        self.error_dict.update({'kP0_'+str(i):self.climber_left_controller.setP(self.PID_dict_pos['kP'], 0)})
        self.error_dict.update({'kP1_'+str(i):self.climber_left_controller.setP(self.PID_dict_vel['kP'], 1)})
        self.error_dict.update({'kI0_'+str(i):self.climber_left_controller.setI(self.PID_dict_pos['kI'], 0)})
        self.error_dict.update({'kI1_'+str(i):self.climber_left_controller.setI(self.PID_dict_vel['kI'], 1)})
        self.error_dict.update({'kD0_'+str(i):self.climber_left_controller.setD(self.PID_dict_pos['kD'], 0)})
        self.error_dict.update({'kD_1'+str(i):self.climber_left_controller.setD(self.PID_dict_vel['kD'], 1)})
        self.error_dict.update({'kFF_0'+str(i):self.climber_left_controller.setFF(self.PID_dict_pos['kFF'], 0)})
        self.error_dict.update({'kFF_1'+str(i):self.climber_left_controller.setFF(self.PID_dict_vel['kFF'], 1)})
        self.error_dict.update({'kIZ_0'+str(i):self.climber_left_controller.setIZone(self.PID_dict_pos['kIz'], 0)})
        self.error_dict.update({'kIZ_1'+str(i):self.climber_left_controller.setIZone(self.PID_dict_vel['kIz'], 1)})
        self.error_dict.update({'MinMax0_'+str(i):self.climber_left_controller.setOutputRange(self.PID_dict_pos['kMinOutput'], self.PID_dict_pos['kMaxOutput'], 0)})
        self.error_dict.update({'MinMax0_'+str(i):self.climber_left_controller.setOutputRange(self.PID_dict_vel['kMinOutput'], self.PID_dict_vel['kMaxOutput'], 1)})
        self.error_dict.update({'Accel0_'+str(i):self.climber_left_controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 0)}) #
        self.error_dict.update({'Accel1_'+str(i):self.climber_left_controller.setSmartMotionMaxVelocity(self.smartmotion_maxvel, 1)}) #
        self.error_dict.update({'Vel0_'+str(i):self.climber_left_controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 0)}) #
        self.error_dict.update({'Vel1_'+str(i):self.climber_left_controller.setSmartMotionMaxAccel(self.smartmotion_maxacc, 1)}) #
        self.climber_left_controller.setSmartMotionAllowedClosedLoopError(1,slotID=0)

        if not pid_only:
            # error_dict.append(controller.restoreFactoryDefaults())
            # self.error_dict.update({'OpenRamp_' + str(i): controller.setOpenLoopRampRate(self.ramp_rate)})
            # self.error_dict.update({'ClosedRamp_' + str(i): controller.setClosedLoopRampRate(self.ramp_rate)})
            # self.error_dict.update({'CurLimit_'+str(i):controller.setSmartCurrentLimit(self.current_limit)})
            self.error_dict.update({'VoltComp_'+str(i):self.climber_left_neo.enableVoltageCompensation(12)})
            # some of these got moved to the controller, not the PID controller
            # self.error_dict.update({'Idle_' + str(i): controller.setIdleMode(rev.IdleMode.kBrake)})
            # defaults are 10, 20, 20 on the frame rates - trying to cut down a bit on CAN bandwidth
            #self.error_dict.update({'PeriodicStatus0_'+str(i):controller.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus0, 20)})
            #self.error_dict.update({'PeriodicStatus1_'+str(i):controller.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus1, 40)})
            #self.error_dict.update({'PeriodicStatus2_'+str(i):controller.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus2, 20)})

        if len(set(self.error_dict)) > 1:
            print('\n*Sparkmax setting*     *Response*')
            for key in sorted(self.error_dict.keys()):
                print(f'     {key:15} \t {self.error_dict[key]}', flush=True)
        else:
            print(f'\n *All SparkMax report {list(set(self.error_dict))[0]}')

        if burn_flash:
            start_time = time.time()

            can_error = self.climber_left_neo.burnFlash()
            print(f'Burn flash on controller {i}: {can_error} {int(1000*(time.time()-start_time)):2d}ms after starting')


    def periodic(self) -> None:

        self.counter += 1

        if self.climber_enable and self.counter % 2 == 0:
            self.current_filter.calculate(self.climber_left_neo.getOutputCurrent())  # update the current filter

        if self.counter % 10 == 0:
            # ten per second updates
            SmartDashboard.putNumber('climber_voltage', self.climber_left_neo.getAppliedOutput() * 12)
            SmartDashboard.putNumber('climber_position', self.climber_left_encoder.getPosition())
            SmartDashboard.putNumber('climber_current', self.current_filter.calculate(self.climber_left_neo.getOutputCurrent()))
            SmartDashboard.putNumber('climber_velocity', self.climber_left_encoder.getVelocity())

            #self.climber_voltage.setValue(self.climber_left_neo.getAppliedOutput())  # does not like setNumber for some reason
            #self.climber_current.setValue(self.current_filter.calculate(self.climber_left_neo.getOutputCurrent()))




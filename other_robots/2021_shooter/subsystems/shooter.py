# shooter to use both in sim and robot mode - sim handles the Sparkmax w/o crashing now but does not simulate well

from wpilib.command import Subsystem
from wpimath import controller
from wpilib import Spark, Encoder, DigitalInput, Talon, SmartDashboard
from commands.shooter_hood_axis import ShooterHoodAxis
import rev

class Shooter(Subsystem):
    # ----------------- INITIALIZATION -----------------------
    def __init__(self, robot):
        super().__init__("shooter")
        self.robot = robot
        self.counter = 0  # used for updating the log
        self.feed_forward = 3.5  # default volts to give the flywheel to get close to setpoint, optional

        # motor controllers
        self.sparkmax_flywheel = rev.CANSparkMax(5, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.spark_hood = Talon(5)
        self.spark_feed = Talon(7)

        # encoders and PID controllers
        self.hood_encoder = Encoder(4, 5)  # generic encoder - we'll have to install one on the 775 motor
        self.hood_encoder.setDistancePerPulse(1/1024)
        self.hood_controller = controller.PIDController(Kp=0.005, Ki=0, Kd=0.0)
        self.hood_setpoint = 5
        self.hood_controller.setSetpoint(self.hood_setpoint)
        self.flywheel_encoder = self.sparkmax_flywheel.getEncoder()  # built-in to the sparkmax/neo
        self.flywheel_controller = self.sparkmax_flywheel.getPIDController()  # built-in PID controller in the sparkmax


        # limit switches, use is TBD
        self.limit_low = DigitalInput(6)
        self.limit_high = DigitalInput(7)

    def initDefaultCommand(self):
        """ When other commands aren't using the drivetrain, allow arcade drive with the joystick. """
        #self.setDefaultCommand(ShooterHoodAxis(self.robot))

    def set_flywheel(self, velocity):
        #self.flywheel_controller.setReference(velocity, rev.ControlType.kVelocity, 0, self.feed_forward)
        self.flywheel_controller.setReference(velocity, rev.ControlType.kSmartVelocity, 0)
        #self.flywheel_controller.setReference(velocity, rev.ControlType.kVelocity, 0)


    def stop_flywheel(self):
        self.flywheel_controller.setReference(0, rev.ControlType.kVoltage)

    def set_feed_motor(self, speed):
        self.spark_feed.set(speed)

    def stop_feed_motor(self):
        self.spark_feed.set(0)

    def set_hood_motor(self, power):
        power_limit = 0.2
        if power > power_limit:
            new_power = power_limit
        elif power < -power_limit:
            new_power = -power_limit
        else:
            new_power = power
        self.spark_hood.set(new_power)

    def change_elevation(self, power):  # open loop approach - note they were wired to be false when contacted
        if power > 0 and self.limit_high.get():
            self.set_hood_motor(power)
        elif power < 0 and self.limit_low.get():
            self.set_hood_motor(power)
        else:
            self.set_hood_motor(0)

    def set_hood_setpoint(self, setpoint):
        self.hood_controller.setSetpoint(setpoint)

    def get_angle(self):
        elevation_minimum = 30  # degrees
        conversion_factor = 1  # encoder units to degrees
        # return elevation_minimum + conversion_factor * self.hood_encoder.getDistance()
        return self.hood_encoder.getDistance()

    def periodic(self) -> None:
        """Perform necessary periodic updates"""
        self.counter += 1
        if self.counter % 5 == 0:
            # pass
            # ten per second updates
            SmartDashboard.putNumber('elevation', self.hood_encoder.getDistance())
            SmartDashboard.putNumber('rpm', self.flywheel_encoder.getVelocity() )

            watch_axis = False
            if watch_axis:
                self.hood_scale = 0.2
                self.hood_offset = 0.0
                power = self.hood_scale * (self.robot.oi.stick.getRawAxis(2) - 0.5) + self.hood_offset
                self.robot.shooter.change_elevation(power)


            maintain_elevation = True
            if maintain_elevation:
                self.error = self.get_angle() - self.hood_setpoint
                pid_out = self.hood_controller.calculate(self.error)
                output = 0.03 + pid_out
                SmartDashboard.putNumber('El PID', pid_out)
                self.change_elevation(output)
        if self.counter % 50 == 0:
            SmartDashboard.putBoolean("hood_low", self.limit_low.get())
            SmartDashboard.putBoolean("hood_high", self.limit_high.get())

            #  print(f'{self.error} {self.hood_setpoint} {self.get_angle()}')

import wpilib
import wpilib.controller
import commands2

import constants


class ElevatorSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.motor = wpilib.Spark(5)
        self.encoder = wpilib.Encoder(4,5)
        self.encoder.setDistancePerPulse(1/2048)

        self.kp = 0.5
        self.ki = 0
        self.kd = 0
        wpilib.SmartDashboard.putNumber('elevator_kp', round(self.kp, 2))
        wpilib.SmartDashboard.putNumber('elevator_ki', round(self.ki, 2))
        wpilib.SmartDashboard.putNumber('elevator_kd', round(self.kd, 2))
        self.controller = wpilib.controller.PIDController(Kp=self.kp, Ki=self.ki, Kd=self.kd)
        self.controlled = False


    def raise_elevator(self, power) -> None:
        """Sends power to the elevator"""
        self.motor.set(power)

    def increment_power(self):
        power = self.motor.getSpeed()
        self.motor.set(power + 0.001)

    def decrement_power(self):
        power = self.motor.getSpeed()
        self.motor.set(power - 0.001)

    def start_controller(self, setpoint) -> None:
        """Sets the control to true"""
        self.kp = wpilib.SmartDashboard.getNumber('elevator_kp', 0)
        self.ki = wpilib.SmartDashboard.getNumber('elevator_ki', 0)
        self.kd = wpilib.SmartDashboard.getNumber('elevator_kd', 0)
        print(f'kp: {self.kp}, ki: {self.ki}, kd: {self.kd}')
        self.controller = wpilib.controller.PIDController(Kp=self.kp, Ki=self.ki, Kd=self.kd)
        self.controller.setSetpoint(setpoint)
        self.controlled = True

    def stop_controller(self) -> None:
        """Sets the control to False"""
        self.controlled = False
        self.motor.set(0)

    def get_height(self):
        """Return the encoder value"""
        return self.encoder.getDistance()

    def periodic(self) -> None:
        if self.controlled: # pretend elevator
            pid_output = self.controller.calculate(self.encoder.getDistance())
            self.motor.set(pid_output)

        wpilib.SmartDashboard.putNumber('elevator', round(self.encoder.getDistance(), 2))
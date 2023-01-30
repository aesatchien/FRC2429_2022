import wpilib
import rev
import math


class MyRobot(wpilib.TimedRobot):


    conversion_factor = 10 * math.pi / 10.71


def autonomousInit(self):
    self.motor_type = rev.MotorType(1)

    self.motor_1 = rev.CANSparkMax(1, self.motor_type)
    self.PID_controller_1 = self.motor_1.getPIDController()
    self.encoder_1 = self.motor_1.getEncoder()
    # self.encoder_1 = rev.CANSparkMax.getEncoder(self.motor_1, self.motorSensor, counts_per_rev=42)
    self.encoder_1.setPositionConversionFactor(self.conversion_factor)

    self.motor_2 = rev.CANSparkMax(2, self.motor_type)
    self.PID_controller_2 = self.motor_2.getPIDController()
    self.encoder_2 = self.motor_2.getEncoder()
    # self.encoder_2 = rev.CANSparkMax.getEncoder(self.motor_2, self.motorSensor, counts_per_rev=42)
    self.encoder_2.setPositionConversionFactor(self.conversion_factor)

    self.PID_controller_1.setReference(20, rev.ControlType.kPosition)
    self.PID_controller_2.setReference(20, rev.ControlType.kPosition)


if __name__ == "__main__":
    wpilib.run(MyRobot)

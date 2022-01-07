import wpilib
import commands2

import constants


class HatchSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()

        self.counter = 0

        # add a compressor if you want to be able to *disable* it - if you make a solenoid it is auto-enabled
        self.compressor = wpilib.Compressor(0)

        self.hatchSolenoid = wpilib.DoubleSolenoid(
            constants.kHatchSolenoidModule, *constants.kHatchSolenoidPorts
        )

    def grabHatch(self) -> None:
        """Grabs the hatch"""
        self.hatchSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def releaseHatch(self) -> None:
        """Releases the hatch"""
        self.hatchSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def toggle_compressor_control(self):
        """Toggle whether the compressor will auto-start (it does by default if a solenoid is instantiated)"""
        if self.compressor.getClosedLoopControl():
            self.compressor.setClosedLoopControl(False)
        else:
            self.compressor.setClosedLoopControl(True)


    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 20 == 0:  # update the dashboard a few times per second
            wpilib.SmartDashboard.putBoolean('pneumatics/compr_controlled', self.compressor.getClosedLoopControl())
            wpilib.SmartDashboard.putBoolean('pneumatics/compr_on', self.compressor.enabled())
            wpilib.SmartDashboard.putNumber('pneumatics/compr_current', self.compressor.getCompressorCurrent())
            wpilib.SmartDashboard.putBoolean('pneumatics/pressure_switch', self.compressor.getPressureSwitchValue())
            wpilib.SmartDashboard.putNumber('pneumatics/solenoid', self.hatchSolenoid.get())
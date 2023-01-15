import commands2
from wpilib import SmartDashboard
from networktables import NetworkTables

class Pixy(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.counter = 0
        self.pixy_table = NetworkTables.getTable('Pixy')

        self.targets_entry = self.pixy_table.getEntry('targets')
        self.distance_entry = self.pixy_table.getEntry('distance')
        self.rotation_entry = self.pixy_table.getEntry('rotation')
        self.sig_entry = self.pixy_table.getEntry('sig')

        self.targets = 0
        self.distance = 0
        self.rotation = 0
        self.sig = 0

    def periodic(self) -> None:
        self.counter += 1

        self.targets = self.targets_entry.getDouble(0)
        self.distance = self.distance_entry.getDouble(0)
        self.rotation = self.rotation_entry.getDouble(0)
        self.sig = self.sig_entry.getDouble(0)

        if self.counter % 10 == 0:
            SmartDashboard.putNumber('/pixy_debug/targets', self.targets)
            SmartDashboard.putNumber('/pixy_debug/distance', self.distance)
            SmartDashboard.putNumber('/pixy_debug/rotation', self.rotation)
            SmartDashboard.putNumber('/pixy_debug/sig', self.sig)

    def getBallValues(self):
        return (self.targets > 0, self.rotation, self.distance)
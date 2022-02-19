import commands2
import networktables
from wpilib import SmartDashboard
from networktables import NetworkTables

class Vision(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.counter = 0
        
        self.ballcam_table = NetworkTables.getTable('BallCam')
        self.fms_info_table = NetworkTables.getTable('FMSInfo')
        self.camera_dict = {'red': {}, 'blue': {}}

        for key in self.camera_dict.keys():
            self.camera_dict[key].update({'targets_entry': self.ballcam_table.getEntry(f"/{key}/targets")})
            self.camera_dict[key].update({'distance_entry': self.ballcam_table.getEntry(f"/{key}/distance")})
            self.camera_dict[key].update({'rotation_entry': self.ballcam_table.getEntry(f"/{key}/rotation")})

        self.targets = 0
        self.distance = 0
        self.rotation = 0
        
        self.team_color = 'red'

    def periodic(self) -> None:
        self.counter += 1

        if (self.fms_info_table.getEntry('IsRedAlliance').getBoolean(True)):
            self.team_color = 'red'
        else:
            self.team_color = 'blue'

        self.targets = self.camera_dict[self.team_color].targets_entry.getNumber(0)
        self.distance = self.camera_dict[self.team_color].distance_entry.getNumber(0)
        self.rotation = self.camera_dict[self.team_color].rotation_entry.getNumber(0)
        
        # update SmartDashboard five times a second
        if self.counter % 10 == 0:
            SmartDashboard.putNumber('/Vision/targets', self.targets)
            SmartDashboard.putNumber('/Vision/distance', self.distance)
            SmartDashboard.putNumber('/Vision/rotation', self.rotation)

    def getBallValues(self):
        return (self.targets > 0, self.rotation, self.distance)
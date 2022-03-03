from commands2 import SubsystemBase
from wpilib import SmartDashboard
from networktables import NetworkTables
from wpilib import DriverStation

class Vision(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Vision')
        self.counter = 0
        
        self.ballcam_table = NetworkTables.getTable('BallCam')
        self.driver_station = DriverStation.getInstance()
        self.camera_dict = {'red': {}, 'blue': {}}

        for key in self.camera_dict.keys():
            self.camera_dict[key].update({'targets_entry': self.ballcam_table.getEntry(f"/{key}/targets")})
            self.camera_dict[key].update({'distance_entry': self.ballcam_table.getEntry(f"/{key}/distance")})
            self.camera_dict[key].update({'rotation_entry': self.ballcam_table.getEntry(f"/{key}/rotation")})

        self.targets = 0
        self.distance = 0
        self.rotation = 0

        # set to red by default
        self.team_color = 'red'

    def periodic(self) -> None:
        self.counter += 1

        # update five times a second
        if self.counter % 10 == 0:
            allianceColor = self.driver_station.getAlliance()
            if allianceColor == DriverStation.Alliance.kRed:
                self.team_color = 'red'
            elif allianceColor == DriverStation.Alliance.kBlue:
                self.team_color = 'blue'

            self.targets = self.camera_dict[self.team_color]['targets_entry'].getDouble(0)
            self.distance = self.camera_dict[self.team_color]['distance_entry'].getDouble(0)
            self.rotation = self.camera_dict[self.team_color]['rotation_entry'].getDouble(0)

            SmartDashboard.putNumber('/Vision/targets', self.targets)
            SmartDashboard.putNumber('/Vision/distance', self.distance)
            SmartDashboard.putNumber('/Vision/rotation', self.rotation)

    def getBallValues(self):
        return (self.targets > 0, self.rotation, self.distance)
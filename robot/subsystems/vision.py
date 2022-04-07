import wpilib
from commands2 import SubsystemBase
from wpilib import SmartDashboard
from networktables import NetworkTables
from wpilib import DriverStation

import constants


class Vision(SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Vision')
        self.counter = 0

        self.hub_offset = -4

        self.ballcam_table = NetworkTables.getTable('BallCam')
        self.driver_station = DriverStation.getInstance()
        self.camera_dict = {'red': {}, 'blue': {}, 'green': {}}

        for key in self.camera_dict.keys():
            self.camera_dict[key].update({'targets_entry': self.ballcam_table.getEntry(f"/{key}/targets")})
            self.camera_dict[key].update({'distance_entry': self.ballcam_table.getEntry(f"/{key}/distance")})
            self.camera_dict[key].update({'rotation_entry': self.ballcam_table.getEntry(f"/{key}/rotation")})

        self.ball_targets = 0
        self.ball_distance = 0
        self.ball_rotation = 0

        self.hub_targets = 0
        self.hub_distance = 0
        self.hub_rotation = 0

        # set to red by default
        self.team_color = 'red'

    def periodic(self) -> None:
        self.counter += 1

        # update five times a second
        if self.counter % 20 == 0:

            if constants.k_is_simulation:
                SmartDashboard.putNumber('match_time', wpilib.Timer.getFPGATimestamp())
            else:
                SmartDashboard.putNumber('match_time', DriverStation.getMatchTime())

            allianceColor = self.driver_station.getAlliance()
            if allianceColor == DriverStation.Alliance.kRed:
                self.team_color = 'red'
            elif allianceColor == DriverStation.Alliance.kBlue:
                self.team_color = 'blue'

            self.ball_targets = self.camera_dict[self.team_color]['targets_entry'].getDouble(0)
            self.ball_distance = self.camera_dict[self.team_color]['distance_entry'].getDouble(0)
            self.ball_rotation = self.camera_dict[self.team_color]['rotation_entry'].getDouble(0)

            # update hub values separately
            self.hub_targets = self.camera_dict['green']['targets_entry'].getDouble(0)
            self.hub_distance = self.camera_dict['green']['distance_entry'].getDouble(0)
            self.hub_rotation = self.camera_dict['green']['rotation_entry'].getDouble(0)

            SmartDashboard.putNumber('/Vision/ball/targets', self.ball_targets)
            SmartDashboard.putNumber('/Vision/ball/distance', self.ball_distance)
            SmartDashboard.putNumber('/Vision/ball/rotation', self.ball_rotation)


            SmartDashboard.putNumber('hub_targets', self.hub_targets)
            if self.hub_targets > 0:
                SmartDashboard.putNumber('hub_distance', self.hub_distance)
                SmartDashboard.putNumber('hub_rotation', self.hub_rotation)
            else:
                SmartDashboard.putNumber('hub_distance', 999)
                SmartDashboard.putNumber('hub_rotation', 999)


    def getBallValues(self):
        return (self.ball_targets > 0, self.ball_rotation, self.ball_distance)

    def getHubValues(self):
        return (self.hub_targets > 0, self.hub_rotation + self.hub_offset , self.hub_distance)

    def getShooterRpmNoHood(self):
        if self.hub_targets > 0:
            if self.hub_distance < 1.4:
                return 2300
            elif self.hub_distance > 3.5:
                return 2900
            else:
                # find RPM based on fit
                return 275 * self.hub_distance + 1958
        else:
            return 2500

    def getShooterHoodRpm(self):
        if self.hub_targets > 0:
            if self.hub_distance < 2.5:
                return 2600
            elif self.hub_distance > 4:
                return 3000
            else:
                return 2486 - 103 * self.hub_distance + 57.1 * self.hub_distance ** 2
        else:
            return 2800
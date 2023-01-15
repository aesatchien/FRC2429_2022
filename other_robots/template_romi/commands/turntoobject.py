
import math
import commands2
from networktables import NetworkTable, NetworkTables
from wpilib import SmartDashboard, Timer
from subsystems.drivetrain import Drivetrain
from commands.turndegrees_ffwd import TurnDegreesFFWD

class TurnToObject(commands2.CommandBase):
    def __init__(self, drive: Drivetrain) -> None:
       
        super().__init__()

        self.drive = drive
            
            
    def initialize(self) -> None:
        """Called when the command is initially scheduled."""
        self.pixy = NetworkTables.getTable('Pixy')
        self.degrees_entry = self.pixy.getNumber('rotation', 10)
        #self.degrees = self.degrees_entry.getNumber()
        TurnDegreesFFWD(speed=0.5, degrees=self.degrees_entry, drive=self.drive)



    def execute(self) -> None:
        """Called every time the scheduler runs while the command is scheduled."""

    def end(self, interrupted: bool) -> None:
        """Called once the command ends or is interrupted."""
        self.drive.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        """Returns true when the command should end"""
        return True

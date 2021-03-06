import commands2
from wpilib import SmartDashboard

class IndexerHold(commands2.CommandBase):

    def __init__(self, container, indexer, voltage=2, shot_time=None, force=None, autonomous=False) -> None:
        super().__init__()
        self.setName('IndexerHold')
        self.indexer = indexer
        self.container = container
        self.voltage = voltage
        self.force = force
        self.addRequirements(indexer)  # commandsv2 version of requirements
        self.on_pulse_time = 0.14  # was 0.15
        self.off_pulse_time = 0.15 # was 0.35  then 0.2 before AVR
        self.direction = 1
        self.autonomous = autonomous  # use this to feed the drive in auto
        self.shot_time = shot_time


    def initialize(self) -> None:
        
        """Called just before this Command runs the first time."""
        self.start_time = self.container.get_enabled_time()
        self.current_time = self.start_time

        print("\n" + f"** Started {self.getName()} with shot_time={self.shot_time} at {self.start_time:2.2f} s **", flush=True)
        # SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        if self.container.co_driver_controller is not None:
            self.direction = -1 if self.container.co_driver_controller.getStartButton() else 1

        # puslse the indexer off and on
        self.current_time = self.container.get_enabled_time() - self.start_time

        if self.current_time % (self.on_pulse_time + self.off_pulse_time) < self.on_pulse_time:
            if not self.indexer.indexer_enabled:
                self.indexer.set_voltage(self.voltage * self.direction)
                # print('shoot')
        else:
            if self.indexer.indexer_enabled:
                self.indexer.stop_motor()
                # print('wait')

        if self.autonomous:
            self.container.robot_drive.feed()

    def isFinished(self) -> bool:
        if self.shot_time is None:
            return False
        else:
            # end if we have completed our set number of cycles
            # return self.current_time > self.cycles * (self.on_pulse_time + self.off_pulse_time)
            return self.current_time > self.shot_time

    def end(self, interrupted: bool) -> None:
        self.indexer.stop_motor()
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

        

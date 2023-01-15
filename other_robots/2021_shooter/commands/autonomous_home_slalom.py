# a group of commands strung together autonomously
from wpilib.command import CommandGroup
from commands.autonomous_rotate import AutonomousRotate
from commands.autonomous_drive_pid import AutonomousDrivePID


class AutonomousSlalom(CommandGroup):
    """ allows for stringing together autonomous commands """

    def __init__(self, robot, timeout=None):
        CommandGroup.__init__(self, name='auto_slalom')
        # slalom
        relative_angles = False
        if relative_angles:
            self.addSequential(AutonomousDrivePID(robot, setpoint=0.8, timeout=3))  # initial move from start
            self.addSequential(AutonomousRotate(robot, setpoint=-55, timeout=3))  # first CCW turn
            self.addSequential(AutonomousDrivePID(robot, setpoint=2.25, timeout=3))  # first diagonal
            self.addSequential(AutonomousRotate(robot, setpoint=55, timeout=3))  # first CW turn on top
            self.addSequential(AutonomousDrivePID(robot, setpoint=3.0, timeout=3))  # cross the top
            self.addSequential(AutonomousRotate(robot, setpoint=55, timeout=3))  # aim down
            self.addSequential(AutonomousDrivePID(robot, setpoint=2.1, timeout=3))  # drive down
            self.addSequential(AutonomousRotate(robot, setpoint=-55, timeout=3))  # turn back
            self.addSequential(AutonomousDrivePID(robot, setpoint=1, timeout=3))  # drive along bottom
            self.addSequential(AutonomousRotate(robot, setpoint=-90, timeout=3))  # point up
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.7, timeout=3))  # drive up
            self.addSequential(AutonomousRotate(robot, setpoint=-90, timeout=3))  # face back to start
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.0, timeout=3))  # cross back over top
            self.addSequential(AutonomousRotate(robot, setpoint=-60, timeout=3))  # CCW point down
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.9, timeout=3))  # drive down diagonal
            self.addSequential(AutonomousRotate(robot, setpoint=65, timeout=3))  # face back to home
            self.addSequential(AutonomousDrivePID(robot, setpoint=3.4, timeout=3))  # cross the bottom
            self.addSequential(AutonomousRotate(robot, setpoint=52, timeout=3))  # point back up
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.9, timeout=3))  # drive up diagonal
            self.addSequential(AutonomousRotate(robot, setpoint=-60, timeout=3))  # CCW turn to finish
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.0, timeout=3))  # drive on finish area
        else:  # use absolute heading
            # slalom with absolute angles E=0, S=90, W=180, N=-90 or 270
            diagonal_distance = 2.1
            self.addSequential(AutonomousDrivePID(robot, setpoint=0.7, timeout=3))  # initial move from start
            self.addSequential(AutonomousRotate(robot, setpoint=-55, absolute=True, timeout=3))  # first CCW turn
            self.addSequential(AutonomousDrivePID(robot, setpoint=diagonal_distance, timeout=3))  # first diagonal
            self.addSequential(AutonomousRotate(robot, setpoint=0, absolute=True, timeout=3))  # first CW turn on top
            self.addSequential(AutonomousDrivePID(robot, setpoint=2.8, timeout=3))  # cross the top
            self.addSequential(AutonomousRotate(robot, setpoint=55, absolute=True, timeout=3))  # aim down
            self.addSequential(AutonomousDrivePID(robot, setpoint=0.9*diagonal_distance, timeout=3))  # drive down
            self.addSequential(AutonomousRotate(robot, setpoint=0, absolute=True, timeout=3))  # turn E
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.0, timeout=3))  # drive along bottom
            self.addSequential(AutonomousRotate(robot, setpoint=-90, absolute=True, timeout=3))  # point up
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.5, timeout=3))  # drive up
            self.addSequential(AutonomousRotate(robot, setpoint=180, absolute=True, timeout=3))  # face back to start
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.2, timeout=3))  # cross back over top
            self.addSequential(AutonomousRotate(robot, setpoint=115, absolute=True, timeout=3))  # SW point down
            self.addSequential(AutonomousDrivePID(robot, setpoint=0.8*diagonal_distance, timeout=3))  # drive down diagonal
            self.addSequential(AutonomousRotate(robot, setpoint=180, absolute=True, timeout=3))  # face back to home
            self.addSequential(AutonomousDrivePID(robot, setpoint=3.3, timeout=3))  # cross the bottom
            self.addSequential(AutonomousRotate(robot, setpoint=-115, absolute=True, timeout=3))  # point back NW
            self.addSequential(AutonomousDrivePID(robot, setpoint= 0.75*diagonal_distance, timeout=3))  # drive up diagonal
            self.addSequential(AutonomousRotate(robot, setpoint=180, absolute=True, timeout=3))  # CCW turn to finish
            self.addSequential(AutonomousDrivePID(robot, setpoint=1.0, timeout=3))  # drive on finish area


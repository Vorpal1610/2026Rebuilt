from enum import auto, Enum
from typing import Optional

from commands2 import Command, Subsystem, cmd
from ntcore import NetworkTableInstance
from wpilib import DriverStation
from wpimath.geometry import Pose3d

from constants import Constants
from subsystems.intake import IntakeSubsystem
from subsystems.swerve import SwerveSubsystem
from subsystems.vision import VisionSubsystem
from subsystems.climber import ClimberSubsystem


class Superstructure(Subsystem):
    """
    The Superstructure is in charge of handling all subsystems to ensure no conflicts between them.
    """

    class Goal(Enum):
        DEFAULT = auto()
        CLIMBING = auto()


    # Map each goal to each subsystem state to reduce code complexity
    _goal_to_states: dict[Goal,
            tuple[
                Optional[VisionSubsystem.SubsystemState]
            ]] = {
        Goal.DEFAULT: (IntakeSubsystem.SubsystemState.STOP, VisionSubsystem.SubsystemState.ALL_ESTIMATES),
        Goal.CLIMBING: (IntakeSubsystem.SubsystemState.STOP, VisionSubsystem.SubsystemState.ALL_ESTIMATES),
       
    }

    def __init__(self, drivetrain: SwerveSubsystem, vision: VisionSubsystem, climber: ClimberSubsystem, intake: IntakeSubsystem) -> None:
        """
        Constructs the superstructure using instance of each subsystem.

        :param drivetrain: Swerve drive base
        :type drivetrain: SwerveSubsystem
        :param vision: Handles all vision estimates
        :type vision: VisionSubsystem
        :param climber: Subsystem that handles the climber
        :type climber: ClimberSubsystem
        :param intake: Subsystem that handles the intake
        :type intake: IntakeSubsystem
        """
        super().__init__()
        self.drivetrain = drivetrain
        self.vision = vision
        self.climber = climber
        self.intake = intake

        self._goal = self.Goal.DEFAULT
        self.set_goal_command(self._goal)

        # table = NetworkTableInstance.getDefault().getTable("Superstructure")
        # self._current_goal_pub = table.getStringTopic("Current Goal").publish()
        # self._component_poses = table.getStructArrayTopic("Components", Pose3d).publish()

    def periodic(self):
        if DriverStation.isDisabled():
            return

        # If climber motor position is at the top position (1 is the placeholder for what the value would actually be), it will go to the full climb state
        if self.climber.get_position() > Constants.ClimberConstants.CLIMB_FULL_THRESHOLD and self.climber.get_current_state() is ClimberSubsystem.SubsystemState.CLIMB_IN:
            self.climber.set_desired_state(ClimberSubsystem.SubsystemState.CLIMB_IN_FULL)
        # TODO add other subsystem periodic functions

    def _set_goal(self, goal: Goal) -> None:
        self._goal = goal

        intake_state = self.intake.get_current_state()

        vision_state = self._goal_to_states.get(goal, (None, None, None, None))
        safety_checks = self._should_enable_safety_checks(intake_state) # TODO pass states that are required for safety checks
        
        if vision_state:
            self.vision.set_desired_state(vision_state)


    def _should_enable_safety_checks(self, intake_state: IntakeSubsystem.SubsystemState) -> bool:
        """ Safety check example of intake being in the frame """
        if intake_state == self.intake.get_current_state():
            return False
        return not (
                self.intake.get_current_state().value < Constants.IntakeConstants.INSIDE_FRAME_ANGLE
        )

    def set_goal_command(self, goal: Goal) -> Command:
        """
        Return a command that sets the superstructure goal to whatever the desired goal is.

        :param goal: The desired goal
        :type goal:  Goal
        :return:     A command that will set the desired goal
        :rtype:      Command
        """
        return cmd.runOnce(lambda: self._set_goal(goal), self)

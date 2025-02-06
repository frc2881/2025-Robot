from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Pose2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation
import core.constants as constants

class AutoPath(Enum):
  Start1_1 = auto()
  Start2_2 = auto()
  Start3_3 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }
    self._selectedAutoCommand = cmd.none()

    AutoBuilder.configure(
      self._robot.localizationService.getRobotPose, 
      self._robot.localizationService.resetRobotPose, 
      self._robot.driveSubsystem.getChassisSpeeds, 
      self._robot.driveSubsystem.drive, 
      constants.Subsystems.Drive.kPathPlannerController,
      constants.Subsystems.Drive.kPathPlannerRobotConfig,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.driveSubsystem
    )

    self._autoCommandChooser = SendableChooser()
    self._autoCommandChooser.setDefaultOption("None", cmd.none)
    
    self._autoCommandChooser.addOption("[1]_1", self.auto_1_1)
    self._autoCommandChooser.addOption("[2]_2", self.auto_2_2)
    self._autoCommandChooser.addOption("[3]_3", self.auto_3_3)

    self._autoCommandChooser.onChange(lambda autoCommand: setattr(self, "_selectedAutoCommand", autoCommand()))
    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)

  def getSelected(self) -> Command:
    return self._selectedAutoCommand

  def _move(self, path: AutoPath) -> Command:
    return AutoBuilder.followPath(self._paths.get(path)).withTimeout(
      constants.Game.Commands.kAutoMoveTimeout
    )
  
  def _getStartingPose(self, position: int) -> Pose2d:
    match position:
      case 1:
        return self._paths.get(AutoPath.Start1_1).getStartingHolonomicPose()
      case 2: 
        return self._paths.get(AutoPath.Start2_2).getStartingHolonomicPose()
      case 3:
        return self._paths.get(AutoPath.Start3_3).getStartingHolonomicPose()
      case _:
        return None

  def moveToStartingPosition(self, position: int) -> Command:
    return AutoBuilder.pathfindToPose(self._getStartingPose(position), constants.Subsystems.Drive.kPathPlannerConstraints)

  def _alignToTarget(self, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return self._robot.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, targetAlignmentLocation).withTimeout(
      constants.Game.Commands.kAutoTargetAlignmentTimeout
    )

  def auto_1_1(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Start1_1),
      self._alignToTarget(TargetAlignmentLocation.Left)
    ).withName("AutoCommands:[1]_1")
  
  def auto_2_2(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Start2_2),
      self._alignToTarget(TargetAlignmentLocation.Left)
    ).withName("AutoCommands:[2]_2")
  
  def auto_3_3(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Start3_3),
      self._alignToTarget(TargetAlignmentLocation.Left)
    ).withName("AutoCommands:[3]_3")
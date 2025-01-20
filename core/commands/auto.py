from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation
import core.constants as constants

class AutoPath(Enum):
  Move1 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }

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
    self._autoCommandChooser.addOption("[0]_1_", self.auto_0_1_)
    SmartDashboard.putData("Robot/Auto/Command", self._autoCommandChooser)

  def getSelected(self) -> Command:
    return self._autoCommandChooser.getSelected()()

  def _move(self, path: AutoPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      self._paths.get(path), 
      constants.Subsystems.Drive.kPathPlannerConstraints
    ).withTimeout(
      constants.Game.Commands.kAutoMoveTimeout
    )
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(self._robot.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Left))

  def auto_0_1_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move1),
      self._alignToTarget()
    ).withName("AutoCommands:[0]_1_")
  
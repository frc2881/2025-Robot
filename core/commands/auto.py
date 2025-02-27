from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Pose2d, Transform2d
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
from lib import logger, utils
from lib.classes import Alliance, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation, TargetPositionType, GamePiece, TargetType
import core.constants as constants

class AutoPath(Enum):
  Start1_1 = auto()
  Start2_2 = auto()
  Start3_3 = auto()
  Pickup1_1 = auto()
  #Pickup2_1 = auto()?
  #Pickup2_2 = auto()?
  Pickup3_2 = auto()
  Pickup4_2 = auto()
  Pickup5_1 = auto()
  Pickup5_2 = auto()
  Pickup6_1 = auto()
  Move1_1 = auto()
  Move1_5 = auto()
  Move1_6 = auto()
  Move2_3 = auto()
  Move2_4 = auto()
  Move2_5 = auto()

class Auto:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }
    self._auto = cmd.none()

    AutoBuilder.configure(
      self._robot.localization.getRobotPose, 
      self._robot.localization.resetRobotPose, 
      self._robot.drive.getChassisSpeeds, 
      self._robot.drive.drive, 
      constants.Subsystems.Drive.kPathPlannerController,
      constants.Subsystems.Drive.kPathPlannerRobotConfig,
      lambda: utils.getAlliance() == Alliance.Red,
      self._robot.drive
    )

    self._autos = SendableChooser()
    self._autos.setDefaultOption("None", cmd.none)
    
    self._autos.addOption("[1]_1", self.auto_1_1)
    self._autos.addOption("[1]_1_6", self.auto_1_1_6)
    self._autos.addOption("[2]_2", self.auto_2_2)
    self._autos.addOption("[3]_3", self.auto_3_3)
    self._autos.addOption("[3]_3_4", self.auto_3_3_4)

    self._autos.onChange(lambda auto: setattr(self, "_auto", auto()))
    SmartDashboard.putData("Robot/Auto", self._autos)

  def get(self) -> Command:
    return self._auto
  
  def _reset(self, path: AutoPath) -> Command:
    return cmd.sequence(
      AutoBuilder.resetOdom(self._paths.get(path).getPathPoses()[0].transformBy(Transform2d(0, 0, self._paths.get(path).getInitialHeading()))),
      cmd.waitSeconds(0.1)
    )
  
  def _move(self, path: AutoPath) -> Command:
    return AutoBuilder.followPath(self._paths.get(path)).withTimeout(constants.Game.Commands.kAutoMoveTimeout)
  
  def _alignToTarget(self, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return self._robot.game.alignRobotToTarget(TargetAlignmentMode.Translation, targetAlignmentLocation)
  
  def _alignForScoring(self) -> Command:
    return self._robot.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL4).until(lambda: self._robot.game.isRobotAlignedToTargetPosition())
  
  def _moveAlignScore(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._move(autoPath).deadlineFor(self._alignForScoring())
      .andThen(self._alignToTarget(targetAlignmentLocation))
      .andThen(self._alignForScoring())
      .andThen(cmd.waitSeconds(0.02))
      .andThen(self._robot.game.score(GamePiece.Coral))
    )
  
  def _moveAlignIntake(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._move(autoPath).deadlineFor(self._robot.game.alignRobotToTargetPosition(TargetPositionType.CoralStation))
      .andThen(self._alignToTarget(targetAlignmentLocation))
      .andThen(self._robot.game.alignRobotToTargetPosition(TargetPositionType.CoralStation))
      .andThen(cmd.waitSeconds(2.0))
    )
  
  def _getStartingPose(self, position: int) -> Pose2d:
    match position:
      case 1: return self._paths.get(AutoPath.Start1_1).getStartingHolonomicPose()
      case 2: return self._paths.get(AutoPath.Start2_2).getStartingHolonomicPose()
      case 3: return self._paths.get(AutoPath.Start3_3).getStartingHolonomicPose()
      case _: return None

  def moveToStartingPosition(self, position: int) -> Command:
    return AutoBuilder.pathfindToPose(
      self._getStartingPose(position), 
      constants.Subsystems.Drive.kPathPlannerConstraints
    ).onlyIf(
      lambda: not utils.isCompetitionMode()
    ).withName("Auto:MoveToStartingPosition")

  def auto_1_1(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start1_1, TargetAlignmentLocation.Left)
    ).withName("Auto:[1]_1")
  
  def auto_1_1_6(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start1_1, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup1_1, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move1_6, TargetAlignmentLocation.Left)
    ).withName("Auto:[1]_1_6")
  
  def auto_1_1_6_6(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start1_1, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup1_1, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move1_6, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup1_1, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move1_6, TargetAlignmentLocation.Right)
    ).withName("Auto:[1]_1_6_6")
  
  def auto_2_2(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2_2, TargetAlignmentLocation.Left)
    ).withName("Auto:[2]_2")
  
  def auto_3_3(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start3_3, TargetAlignmentLocation.Right)
    ).withName("Auto:[3]_3")
  
  def auto_3_3_4(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start3_3, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup3_2, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move2_4, TargetAlignmentLocation.Left)
    ).withName("Auto:[3]_3_4")
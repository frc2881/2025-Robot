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
from core.classes import TargetAlignmentLocation, TargetPositionType
import core.constants as constants

class AutoPath(Enum):
  Start1_1 = auto()
  Start2L_2 = auto()
  Start2R_2 = auto()
  Start3_3 = auto()
  Pickup1_1 = auto()
  Pickup2_1 = auto()
  Pickup2_2 = auto()
  Pickup3_2 = auto()
  Pickup4_2 = auto()
  Pickup6_1 = auto()
  Move1_2 = auto()
  Move1_6L = auto()
  Move1_6R = auto()
  Move2_2 = auto()
  Move2_4L = auto()
  Move2_4R = auto()

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
    
    self._autos.addOption("[3]_344", self.auto_3_344)
    self._autos.addOption("[1]_166", self.auto_1_166)
    self._autos.addOption("[2R]_222", self.auto_2R_222)
    self._autos.addOption("[2L]_212", self.auto_2L_212)
    self._autos.addOption("[2R]_22", self.auto_2R_22)
    self._autos.addOption("[2L]_21", self.auto_2L_21)
    self._autos.addOption("[2L]_2", self.auto_2L_2)

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
    return self._robot.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL4)
  
  def _moveAlignScore(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      cmd.sequence(
        self._move(autoPath),
        self._alignToTarget(targetAlignmentLocation))
      .deadlineFor(self._alignForScoring())
      .andThen(cmd.waitSeconds(0.2).andThen(self._robot.game.scoreCoral()))
    )
  
  def _moveAlignIntake(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._robot.game.alignRobotToTargetPosition(TargetPositionType.CoralStation).alongWith(
        self._move(autoPath).andThen(
          self._alignToTarget(targetAlignmentLocation)
        )
      ).until(lambda: self._robot.game.isGripperHolding())
    )
  
  def _getStartingPose(self, position: int) -> Pose2d:
    match position:
      case 1: return self._paths.get(AutoPath.Start1_1).getStartingHolonomicPose()
      case 2: return self._paths.get(AutoPath.Start2L_2).getStartingHolonomicPose()
      case 3: return self._paths.get(AutoPath.Start3_3).getStartingHolonomicPose()
      case _: return None

  def moveToStartingPosition(self, position: int) -> Command:
    return AutoBuilder.pathfindToPose(
      self._getStartingPose(position), 
      constants.Subsystems.Drive.kPathPlannerConstraints
    ).onlyIf(
      lambda: not utils.isCompetitionMode()
    ).withName("Auto:MoveToStartingPosition")
  
  def auto_3_344(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start3_3, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup3_2, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move2_4R, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup4_2, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move2_4L, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup4_2, TargetAlignmentLocation.Center)
    ).withName("Auto:[3]_344")

  def auto_1_166(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start1_1, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup1_1, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move1_6L, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup6_1, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.Move1_6R, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup6_1, TargetAlignmentLocation.Center)
    ).withName("Auto:[1]_166")
  
  def auto_2R_222(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2R_2, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup2_2, TargetAlignmentLocation.Left),
      self._moveAlignScore(AutoPath.Move2_2, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup2_2, TargetAlignmentLocation.Left)
    ).withName("Auto:[2R]_222")

  def auto_2L_212(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2L_2, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup2_1, TargetAlignmentLocation.Right),
      self._moveAlignScore(AutoPath.Move1_2, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup2_1, TargetAlignmentLocation.Right)
    ).withName("Auto:[2L]_212")

  def auto_2R_22(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2R_2, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup2_2, TargetAlignmentLocation.Left)
    ).withName("Auto:[2R]_22")

  def auto_2L_2(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2L_2, TargetAlignmentLocation.Left)
    ).withName("Auto:[2L]_2")
  
  def auto_2L_21(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2L_2, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup2_1, TargetAlignmentLocation.Right)
    ).withName("Auto:[2L]_21")

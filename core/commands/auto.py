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
  Pickup1_A = auto()
  Pickup2_A = auto()
  Pickup2_B = auto()
  Pickup3_B = auto()
  Pickup4_B = auto()
  Pickup6_A = auto()
  Move2L = auto()
  Move2R = auto()
  MoveA_2 = auto()
  MoveA_6L = auto()
  MoveA_6R = auto()
  MoveB_2 = auto()
  MoveB_4L = auto()
  MoveB_4R = auto()

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
    
    self._autos.addOption("[3]_3_4_4_", self.auto_3_344)
    self._autos.addOption("[1]_1_6_6_", self.auto_1_166)
    self._autos.addOption("[2R]_2B2", self.auto_2R_2B2)
    self._autos.addOption("[2L]_2A2", self.auto_2L_2A2)
    self._autos.addOption("[2R]_2B", self.auto_2R_2B)
    self._autos.addOption("[2L]_2A", self.auto_2L_2A)
    self._autos.addOption("[2R]", self.auto_2RMove)
    self._autos.addOption("[2L]", self.auto_2LMove)
    self._autos.addOption("[2L]2", self.auto_2L)

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
  
  def _moveAlignScore(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation, waitTime: float = 0.1) -> Command:
    return (
      (self._move(autoPath).andThen(self._alignToTarget(targetAlignmentLocation)).andThen(cmd.waitSeconds(waitTime)))
      .deadlineFor(self._robot.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL4))
      .andThen(self._robot.game.scoreCoral())
    )

  def _moveAlignIntake(self, autoPath: AutoPath, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return (
      self._robot.game.alignRobotToTargetPosition(TargetPositionType.CoralStation)
      .alongWith(self._move(autoPath).andThen(self._alignToTarget(targetAlignmentLocation)))
      .until(lambda: self._robot.game.isGripperHolding())
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
      self._moveAlignScore(AutoPath.Start3_3, TargetAlignmentLocation.Left, 0.15),
      self._moveAlignIntake(AutoPath.Pickup3_B, TargetAlignmentLocation.Center), 
      self._moveAlignScore(AutoPath.MoveB_4R, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup4_B, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.MoveB_4L, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup4_B, TargetAlignmentLocation.Center)
    ).withName("Auto:[3]_344")

  def auto_1_166(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start1_1, TargetAlignmentLocation.Right, 0.15),
      self._moveAlignIntake(AutoPath.Pickup1_A, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.MoveA_6L, TargetAlignmentLocation.Left),
      self._moveAlignIntake(AutoPath.Pickup6_A, TargetAlignmentLocation.Center),
      self._moveAlignScore(AutoPath.MoveA_6R, TargetAlignmentLocation.Right),
      self._moveAlignIntake(AutoPath.Pickup6_A, TargetAlignmentLocation.Center)
    ).withName("Auto:[1]_166")
  
  def auto_2R_2B2(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2R_2, TargetAlignmentLocation.Right, 0.15),
      self._moveAlignIntake(AutoPath.Pickup2_B, TargetAlignmentLocation.Left),
      self._moveAlignScore(AutoPath.MoveB_2, TargetAlignmentLocation.Left)
    ).withName("Auto:[2R]_2B2")

  def auto_2L_2A2(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2L_2, TargetAlignmentLocation.Left, 0.15),
      self._moveAlignIntake(AutoPath.Pickup2_A, TargetAlignmentLocation.Right),
      self._moveAlignScore(AutoPath.MoveA_2, TargetAlignmentLocation.Right)
    ).withName("Auto:[2L]_2A2")

  def auto_2R_2B(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2R_2, TargetAlignmentLocation.Right, 0.15),
      self._moveAlignIntake(AutoPath.Pickup2_B, TargetAlignmentLocation.Left)
    ).withName("Auto:[2R]_2B")
  
  def auto_2L_2A(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2L_2, TargetAlignmentLocation.Left, 0.15),
      self._moveAlignIntake(AutoPath.Pickup2_A, TargetAlignmentLocation.Right)
    ).withName("Auto:[2L]_2A")
  
  def auto_2RMove(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2R_2, TargetAlignmentLocation.Right, 0.15),
      cmd.waitSeconds(2.0),
      self._move(AutoPath.Move2R).deadlineFor(self._robot.game.alignRobotToTargetPosition(TargetPositionType.CoralStation))
    ).withName("Auto:[2R]")

  def auto_2LMove(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2L_2, TargetAlignmentLocation.Left, 0.15),
      cmd.waitSeconds(2.0),
      self._move(AutoPath.Move2L).deadlineFor(self._robot.game.alignRobotToTargetPosition(TargetPositionType.CoralStation))
    ).withName("Auto:[2L]")

  def auto_2L(self) -> Command:
    return cmd.sequence(
      self._moveAlignScore(AutoPath.Start2L_2, TargetAlignmentLocation.Left, 0.15)
    ).withName("Auto:[2L]_2")

from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation, TargetType, TargetPositionType, GamePiece
import core.constants as constants

class GameCommands:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

  def alignRobotToTargetCommand(
      self, 
      targetAlignmentMode: TargetAlignmentMode, 
      targetAlignmentLocation: TargetAlignmentLocation = TargetAlignmentLocation.Default, 
      targetType: TargetType = TargetType.Default
    ) -> Command:
    return self._robot.driveSubsystem.alignToTargetCommand(
      self._robot.localizationService.getRobotPose, 
      self._robot.localizationService.getTargetPose,
      targetAlignmentMode,
      targetAlignmentLocation,
      targetType
    ).until(
      lambda: self._robot.driveSubsystem.isAlignedToTarget()
    ).withTimeout(
      constants.Game.Commands.kTargetAlignmentTimeout
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:AlignRobotToTarget")
  
  def alignRobotToTargetPositionCommand(self, targetPositionType: TargetPositionType) -> Command:
    return cmd.parallel(
      self._robot.elevatorSubsystem.alignToPositionCommand(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
      self._robot.armSubsystem.alignToPositionCommand(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm),
      self._robot.wristSubsystem.setPositionCommand(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)
    ).until(
      lambda: (
        self._robot.elevatorSubsystem.isAlignedToPosition() and 
        self._robot.armSubsystem.isAlignedToPosition() and 
        self._robot.wristSubsystem.isAlignedToPosition()
      )
    ).withTimeout(
      constants.Game.Commands.kTargetPositionAlignmentTimeout
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short)
    ).withName("GameCommands:AlignRobotToTargetPosition")
  
  def intakeCommand(self, gamePieceType: GamePiece) -> Command:
    return cmd.either(
      self._robot.handSubsystem.runGripperCommand().until(lambda: self._robot.handSubsystem._isGripperHolding()),
      self._robot.handSubsystem.runSuctionCommand().until(lambda: self._robot.handSubsystem._isSuctionHolding()),
      lambda: gamePieceType == GamePiece.Coral
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Both, ControllerRumblePattern.Short)
    ).withName("GameCommands:IntakeCommand")
  
  def scoreCommand(self, gamePieceType: GamePiece) -> Command:
    return cmd.either(
      self._robot.handSubsystem.releaseGripperCommand().until(lambda: not self._robot.handSubsystem._isGripperEnabled()),
      self._robot.handSubsystem.releaseSuctionCommand().until(lambda: not self._robot.handSubsystem._isSuctionEnabled()),
      lambda: gamePieceType == GamePiece.Coral
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Both, ControllerRumblePattern.Short)
    ).withName("GameCommands:ScoreCommand")

  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self._robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self._robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")

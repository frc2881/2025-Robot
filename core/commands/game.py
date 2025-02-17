from typing import TYPE_CHECKING
import math
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation, TargetType, TargetPositionType, GamePiece, WristPosition
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
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
    ).withName("GameCommands:AlignRobotToTarget")
  
  def alignRobotToTargetPositionCommand(self, targetPositionType: TargetPositionType) -> Command:
    return cmd.sequence(
      self._robot.elevatorSubsystem.alignToPositionCommand(
        constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator
      ).until(lambda: self._robot.elevatorSubsystem.isAlignedToPosition()),
      self._robot.armSubsystem.alignToPositionCommand(
        constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm
      ).until(lambda: self._robot.armSubsystem.isAlignedToPosition()),
      self._robot.wristSubsystem.setPositionCommand(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short)
    ).withName("GameCommands:AlignRobotToTargetPosition")
  
  def alignRobotToTargetPositionReefCoralL4Command(self) -> Command:
    return cmd.either(
      self.alignRobotToTargetPositionCommand(TargetPositionType.ReefCoralL4Score),
      self.alignRobotToTargetPositionCommand(TargetPositionType.ReefCoralL4Ready),
      lambda: math.isclose(
        self._robot.elevatorSubsystem.getPosition().lowerStage, 
        constants.Game.Field.Targets.kTargetPositions[TargetPositionType.ReefCoralL4Ready].elevator.lowerStage, 
        abs_tol=1.0
      )
    )
  
  def intakeCoralCommand(self) -> Command: 
    return cmd.parallel(
      self.alignRobotToTargetPositionCommand(TargetPositionType.CoralStation),
      self.intakeCommand(GamePiece.Coral)
    ).andThen(
      cmd.parallel(
        self._robot.wristSubsystem.setPositionCommand(WristPosition.Up),
        self.rumbleControllersCommand(ControllerRumbleMode.Both, ControllerRumblePattern.Short)
      ) 
    )

  def intakeCommand(self, gamePieceType: GamePiece) -> Command:
    return cmd.either(
      self._robot.handSubsystem.runGripperCommand(),
      self._robot.handSubsystem.runSuctionCommand(),
      lambda: gamePieceType == GamePiece.Coral
    ).withName("GameCommands:IntakeCommand")
  
  def ejectCommand(self, gamePieceType: GamePiece) -> Command:
    return cmd.either(
      self._robot.handSubsystem.releaseGripperCommand(),
      self._robot.handSubsystem.releaseSuctionCommand(),
      lambda: gamePieceType == GamePiece.Coral
    ).withName("GameCommands:EjectCommand")
  
  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self._robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self._robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")

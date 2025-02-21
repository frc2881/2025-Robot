from typing import TYPE_CHECKING
import math
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import Position, Value, TargetAlignmentMode, ControllerRumbleMode, ControllerRumblePattern
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
      self.rumbleControllersCommand(ControllerRumbleMode.Driver)
    ).withName("GameCommands:AlignRobotToTarget")
  
  def alignRobotToTargetPositionCommand(self, targetPositionType: TargetPositionType) -> Command:
    return cmd.parallel(
      self._robot.wristSubsystem.setPositionCommand(Position.Up),
      self._robot.armSubsystem.alignToPositionCommand(Value.min)
    ).until(
      lambda: (self._robot.wristSubsystem.isAlignedToPosition() and self._robot.armSubsystem.isAlignedToPosition())
    ).andThen(
      cmd.parallel(
        self._robot.elevatorSubsystem.alignToPositionCommand(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
        cmd.waitSeconds(1.0).andThen(
          cmd.parallel(
            cmd.waitUntil(lambda: self._robot.elevatorSubsystem.isAlignedToPosition()).andThen(
              self._robot.armSubsystem.alignToPositionCommand(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm)
            ),
            cmd.waitUntil(lambda: self._robot.armSubsystem.isAlignedToPosition()).andThen(
              self._robot.wristSubsystem.setPositionCommand(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)
            )
          )
        )
      ).until(
        lambda: (
          self._robot.elevatorSubsystem.isAlignedToPosition() and 
          self._robot.armSubsystem.isAlignedToPosition() and 
          self._robot.wristSubsystem.isAlignedToPosition()
        )
      )
    ).withName("GameCommands:AlignRobotToTargetPosition")
  
  def alignRobotForScoringCommand(
      self, 
      readyTargetPositionType: TargetPositionType, 
      scoreTargetPositionType: TargetPositionType
    ) -> Command:
    return cmd.either(
      self.alignRobotToTargetPositionCommand(readyTargetPositionType),
      self.alignRobotToTargetPositionCommand(scoreTargetPositionType).deadlineFor(
        self._robot.handSubsystem.runGripperCommand().onlyIf(
          lambda: scoreTargetPositionType in [ 
            TargetPositionType.ReefCoralL4Score,
            TargetPositionType.ReefCoralL3Score,
            TargetPositionType.ReefCoralL2Score,
            TargetPositionType.ReefCoralL4Score
          ]
        )
      ),
      lambda: not self._robot.driveSubsystem.isAlignedToTarget()
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Operator)
    ).withName("GameCommands:AlignRobotToTargetPositionReefCoral")
  
  def intakeCommand(self, gamePiece: GamePiece) -> Command:
    return cmd.either(
      self._robot.handSubsystem.runGripperCommand().until(lambda: self._robot.handSubsystem.isGripperHolding()),
      self._robot.handSubsystem.runSuctionCommand().until(lambda: self._robot.handSubsystem.isSuctionHolding()),
      lambda: gamePiece == GamePiece.Coral
    ).withName("GameCommands:Intake")

  def intakeCoralCommand(self) -> Command: 
    return cmd.parallel(
      self.alignRobotToTargetPositionCommand(TargetPositionType.CoralStation),
      self.intakeCommand(GamePiece.Coral)
    ).andThen(
      cmd.parallel(
        self._robot.wristSubsystem.setPositionCommand(Position.Up),
        self.rumbleControllersCommand(ControllerRumbleMode.Driver)
      )
    ).withName("GameCommands:IntakeCoral")
  
  def intakeAlgaeCommand(self, targetPositionType: TargetPositionType) -> Command: 
    return cmd.parallel(
      self.alignRobotToTargetPositionCommand(targetPositionType),
      self.intakeCommand(GamePiece.Algae)
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Both)
    ).withName("GameCommands:IntakeAlgae")

  def scoreCommand(self, gamePiece: GamePiece) -> Command:
    return cmd.either(
      self._robot.handSubsystem.releaseGripperCommand(),
      self._robot.handSubsystem.releaseSuctionCommand(),
      lambda: gamePiece == GamePiece.Coral
    ).andThen(
      self.rumbleControllersCommand(ControllerRumbleMode.Driver)
    ).withName("GameCommands:Score")
  
  def rumbleControllersCommand(
      self, 
      mode: ControllerRumbleMode = ControllerRumbleMode.Both, 
      pattern: ControllerRumblePattern = ControllerRumblePattern.Short
    ) -> Command:
    return cmd.parallel(
      self._robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Operator),
      self._robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Driver)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")

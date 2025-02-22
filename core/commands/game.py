from typing import TYPE_CHECKING
import math
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import Position, Value, TargetAlignmentMode, ControllerRumbleMode, ControllerRumblePattern
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation, TargetType, TargetPositionType, GamePiece
import core.constants as constants

class Game:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

  def alignRobotToTarget(
      self, 
      targetAlignmentMode: TargetAlignmentMode, 
      targetAlignmentLocation: TargetAlignmentLocation = TargetAlignmentLocation.Default, 
      targetType: TargetType = TargetType.Default
    ) -> Command:
    return self._robot.drive.alignToTarget(
      self._robot.localization.getRobotPose, 
      self._robot.localization.getTargetPose,
      targetAlignmentMode,
      targetAlignmentLocation,
      targetType
    ).until(
      lambda: self._robot.drive.isAlignedToTarget()
    ).withTimeout(
      constants.Game.Commands.kTargetAlignmentTimeout
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Driver)
    ).withName("Game:AlignRobotToTarget")
  
  def alignRobotToTargetPosition(self, targetPositionType: TargetPositionType) -> Command:
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
      self._robot.arm.setPosition(Value.min).until(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm)
      ),
      cmd.waitSeconds(0.3).andThen(
        self._robot.wrist.setPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist).until(lambda: self._robot.elevator.isAlignedToPosition())).andThen(
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)
      )
    ).withName("Game:AlignRobotToTargetPosition")
  
  def alignRobotForScoring(
      self, 
      readyTargetPositionType: TargetPositionType, 
      scoreTargetPositionType: TargetPositionType
    ) -> Command:
    return cmd.either(
      self.alignRobotToTargetPosition(readyTargetPositionType),
      self.alignRobotToTargetPosition(scoreTargetPositionType).deadlineFor(
        self._robot.hand.runGripper().onlyIf(
          lambda: scoreTargetPositionType in [ 
            TargetPositionType.ReefCoralL4Score,
            TargetPositionType.ReefCoralL3Score,
            TargetPositionType.ReefCoralL2Score,
            TargetPositionType.ReefCoralL4Score
          ]
        )
      ),
      lambda: not self._robot.drive.isAlignedToTarget()
    ).deadlineFor(
      cmd.waitUntil(
        lambda: (
          self._robot.elevator.isAlignedToPosition() and
          self._robot.arm.isAlignedToPosition() and
          self._robot.wrist.isAlignedToPosition()
        )
      ).andThen(
        self.rumbleControllers(ControllerRumbleMode.Operator)
      )
    ).withName("Game:AlignRobotForScoring")
  
  def intake(self, gamePiece: GamePiece) -> Command:
    return cmd.either(
      self._robot.hand.runGripper().until(lambda: self._robot.hand.isGripperHolding()),
      self._robot.hand.runSuction().until(lambda: self._robot.hand.isSuctionHolding()),
      lambda: gamePiece == GamePiece.Coral
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Both)
    ).withName("Game:Intake")

  def intakeCoral(self) -> Command:
    return self.alignRobotToTargetPosition(
      TargetPositionType.CoralStation
    ).alongWith(
      self.intake(GamePiece.Coral)
    ).withName("Game:IntakeCoral")

  def intakeAlgae(self, targetPositionType: TargetPositionType) -> Command: 
    return self.alignRobotToTargetPosition(
      targetPositionType
    ).alongWith(
      self.intake(GamePiece.Algae)
    ).withName("Game:IntakeAlgae")

  def score(self, gamePiece: GamePiece) -> Command:
    return cmd.either(
      self._robot.hand.releaseGripper(),
      self._robot.hand.releaseSuction(),
      lambda: gamePiece == GamePiece.Coral
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Driver)
    ).withName("Game:Score")
  
  def rumbleControllers(
      self, 
      mode: ControllerRumbleMode = ControllerRumbleMode.Both, 
      pattern: ControllerRumblePattern = ControllerRumblePattern.Short
    ) -> Command:
    return cmd.parallel(
      self._robot.driver.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Operator),
      self._robot.operator.rumble(pattern).onlyIf(lambda: mode != ControllerRumbleMode.Driver)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("Game:RumbleControllers")

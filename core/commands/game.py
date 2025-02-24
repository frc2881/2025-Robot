from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import Value, TargetAlignmentMode, ControllerRumbleMode, ControllerRumblePattern
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import Position, TargetAlignmentLocation, TargetPositionType, TargetPosition, GamePiece
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
      targetAlignmentLocation: TargetAlignmentLocation
    ) -> Command:
    return self._robot.drive.alignToTarget(
      self._robot.localization.getRobotPose, 
      lambda: self._robot.localization.getTargetPose(targetAlignmentLocation, self._robot.elevator.isReefCoralL4Ready()),
      targetAlignmentMode
    ).until(
      lambda: self._robot.drive.isAlignedToTarget()
    ).withTimeout(
      constants.Game.Commands.kTargetAlignmentTimeout
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Both)
    ).withName("Game:AlignRobotToTarget")

  def alignRobotToTargetPosition(self, targetPositionType: TargetPositionType) -> Command:
    return cmd.either(
      self._alignRobotToTargetPosition(TargetPositionType.ReefCoralL4Ready),
      self._alignRobotToTargetPosition(targetPositionType),
      lambda: targetPositionType == TargetPositionType.ReefCoralL4 and not self._robot.elevator.isReefCoralL4Ready()
    ).withName("Game:AlignRobotToTargetPosition")

  def _alignRobotToTargetPosition(self, targetPositionType: TargetPositionType) -> Command:
    return cmd.either(
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
        self._robot.arm.setPosition(Value.min).until(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm)
        ),
        self._robot.wrist.setPosition(Position.Up).until(lambda: self._robot.arm.isAlignedToPosition()).andThen(
          self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)
        ),
        self._robot.hand.runGripper()
      ),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm)),
        cmd.waitUntil(lambda: self._robot.arm.isAlignedToPosition()).andThen(self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)),
        cmd.either(
          self._robot.hand.runGripper(),
          cmd.either(
            self._robot.hand.runSuction(),
            cmd.none(),
            lambda: targetPositionType in [
              TargetPositionType.ReefAlgaeL3,
              TargetPositionType.ReefAlgaeL2 
            ]
          ),
          lambda: targetPositionType in [
            TargetPositionType.ReefCoralL3,
            TargetPositionType.ReefCoralL2,
            TargetPositionType.ReefCoralL1  
          ]
        )
      ),
      lambda: targetPositionType in [ TargetPositionType.ReefCoralL4, TargetPositionType.CoralStation ]
    )
  
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

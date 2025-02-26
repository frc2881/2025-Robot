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
    ).withName(f'Game:AlignRobotToTarget:{ targetAlignmentMode.name }:{ targetAlignmentLocation.name }')

  def alignRobotToTargetPosition(self, targetPositionType: TargetPositionType) -> Command:
    return cmd.select(
      {
        TargetPositionType.CoralStation: self._alignRobotToTargetPositionArmSafety(TargetPositionType.CoralStation),
        TargetPositionType.ReefCoralL4: 
          cmd.either(
            self._alignRobotToTargetPositionArmSafety(TargetPositionType.ReefCoralL4),
            self._alignRobotToTargetPositionParallel(TargetPositionType.ReefCoralL4Ready),
            lambda: self._robot.elevator.isReefCoralL4Ready()
          ),
        TargetPositionType.ReefCoralL3: self._alignRobotToTargetPositionParallel(TargetPositionType.ReefCoralL3),
        TargetPositionType.ReefCoralL2: self._alignRobotToTargetPositionParallel(TargetPositionType.ReefCoralL2),
        TargetPositionType.ReefCoralL1: self._alignRobotToTargetPositionParallel(TargetPositionType.ReefCoralL1),
        TargetPositionType.ReefAlgaeL3: self._alignRobotToTargetPositionParallel(TargetPositionType.ReefAlgaeL3),
        TargetPositionType.ReefAlgaeL2: self._alignRobotToTargetPositionParallel(TargetPositionType.ReefAlgaeL2),
        TargetPositionType.CageEntry: self._alignRobotToTargetPositionCageEntry()
      }, 
      lambda: targetPositionType
    ).withName(f'Game:AlignRobotToTargetPosition:{ targetPositionType.name }')
  
  def _alignRobotToTargetPositionParallel(self, targetPositionType: TargetPositionType):
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
      cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm)
      ),
      cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)
      ),
      cmd.either(
        self._intakeAligned(GamePiece.Algae), 
        self._intakeAligned(GamePiece.Coral), 
        lambda: targetPositionType in [ TargetPositionType.ReefAlgaeL3, TargetPositionType.ReefAlgaeL2 ]
      )
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Operator))
    )

  def _alignRobotToTargetPositionArmSafety(self, targetPositionType: TargetPositionType):
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
      self._robot.arm.setPosition(Value.min).until(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm)
      ),
      self._robot.wrist.setPosition(Position.Up).until(lambda: self._robot.arm.isAlignedToPosition()).andThen(
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist)
      ),
      self._intakeAligned(GamePiece.Coral)
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Operator))
    )
  
  def _alignRobotToTargetPositionCageEntry(self) -> Command:
    return cmd.sequence(
      self._robot.shield.setPosition(Position.Open),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageEntry].elevator),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageEntry].arm)
        ),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageEntry].wrist)
        )
      )
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
    )

  def isRobotAlignedToTargetPosition(self) -> bool:
    return self._robot.elevator.isAlignedToPosition() and self._robot.arm.isAlignedToPosition() and self._robot.wrist.isAlignedToPosition()

  def _intakeAligned(self, gamePiece: GamePiece) -> Command:
    return cmd.either(
      self._robot.hand.runGripper(),
      self._robot.hand.runSuction(),
      lambda: gamePiece == GamePiece.Coral
    )
  
  def intakeManual(self, gamePiece: GamePiece) -> Command:
    return cmd.either(
      self._robot.hand.runGripper(),
      self._robot.hand.runSuction(),
      lambda: gamePiece == GamePiece.Coral
    ).alongWith(
      self._robot.wrist.refreshPosition()
    ).withName(f'Game:IntakeManual:{ gamePiece.name }')
  
  def score(self, gamePiece: GamePiece) -> Command:
    return cmd.either(
      self._robot.hand.releaseGripper(),
      self._robot.hand.releaseSuction(),
      lambda: gamePiece == GamePiece.Coral
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Driver)
    ).withName(f'Game:Score:{ gamePiece.name }')
  
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
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')

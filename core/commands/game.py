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

  def alignRobotToTarget(self, targetAlignmentMode: TargetAlignmentMode, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return self._robot.drive.alignToTarget(
      self._robot.localization.getRobotPose, 
      lambda: self._robot.localization.getTargetPose(targetAlignmentLocation, self._robot.elevator.isReefCoralL4()),
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
        TargetPositionType.ReefCoralL4: self._alignRobotToTargetPosition(TargetPositionType.ReefCoralL4),
        TargetPositionType.ReefCoralL3: self._alignRobotToTargetPosition(TargetPositionType.ReefCoralL3),
        TargetPositionType.ReefCoralL2: self._alignRobotToTargetPosition(TargetPositionType.ReefCoralL2),
        TargetPositionType.ReefCoralL1: self._alignRobotToTargetPosition(TargetPositionType.ReefCoralL1),
        TargetPositionType.ReefAlgaeL3: self._alignRobotToTargetPosition(TargetPositionType.ReefAlgaeL3),
        TargetPositionType.ReefAlgaeL2: self._alignRobotToTargetPosition(TargetPositionType.ReefAlgaeL2),
        TargetPositionType.CoralStation: self._alignRobotToTargetPositionCoralStation(),
        TargetPositionType.CageDeepClimb: self._alignRobotToTargetPositionCageDeepClimb()
      }, 
      lambda: targetPositionType
    ).withName(f'Game:AlignRobotToTargetPosition:{ targetPositionType.name }')
  
  def _alignRobotToTargetPosition(self, targetPositionType: TargetPositionType):
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator, isParallel = True),
      # TODO: remove the wait on elevator to move arm/wrist in parallel?
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

  def _alignRobotToTargetPositionCoralStation(self):
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].elevator, isParallel = False),
      self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].arm),
      self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].wrist)
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self._intakeAligned(GamePiece.Coral))
    )
  
  def _alignRobotToTargetPositionCageDeepClimb(self) -> Command:
    return cmd.sequence(
      self._robot.shield.setPosition(Position.Open),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].elevator, isParallel = False),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].arm)
        ),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].wrist)
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

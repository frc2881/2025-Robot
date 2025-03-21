from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import TargetAlignmentMode, ControllerRumbleMode, ControllerRumblePattern
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import Position, TargetAlignmentLocation, TargetPositionType
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
      self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm),
      self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist),
      self._robot.hand.runGripper().unless(
        lambda: targetPositionType in [ TargetPositionType.ReefAlgaeL3, TargetPositionType.ReefAlgaeL2 ]
      )
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
    )

  def _alignRobotToTargetPositionCoralStation(self):
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].elevator, isParallel = False),
      self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].arm),
      self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].wrist),
      self._robot.hand.runGripper()
    ).alongWith(
      cmd.waitUntil(lambda: self.isIntakeHolding()).andThen(self.rumbleControllers(ControllerRumbleMode.Driver))
    )
  
  def _alignRobotToTargetPositionCageDeepClimb(self) -> Command:
    return cmd.sequence(
      cmd.waitSeconds(0.5),
      self._robot.shield.setPosition(Position.Open),
      cmd.runOnce(lambda: self._robot.elevator.setUpperStageSoftLimitsEnabled(False)),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].elevator, isParallel = False),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].wrist)
        ),
        cmd.waitUntil(lambda: self._robot.wrist.isAlignedToPosition()).andThen(
          self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].arm)
        )
      )
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
    ).finallyDo(
      lambda end: self._robot.elevator.setUpperStageSoftLimitsEnabled(True)
    )

  def isRobotAlignedToTargetPosition(self) -> bool:
    return (
      self._robot.elevator.isAlignedToPosition() and 
      self._robot.arm.isAlignedToPosition() and 
      self._robot.wrist.isAlignedToPosition()
    )

  def intake(self) -> Command:
    return self._robot.hand.runGripper(
      isManual = True
    ).alongWith(
      self._robot.wrist.refreshPosition()
    ).withName("Game:IntakeManual")
  
  def isIntakeHolding(self) -> bool:
    return self._robot.hand.isGripperHolding()

  def score(self) -> Command:
    return self._robot.hand.releaseGripper().andThen(
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
    ).withName(f'Game:RumbleControllers:{ mode.name }:{ pattern.name }')

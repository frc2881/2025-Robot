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

    self._isFunnelReady = False

  def alignRobotToTarget(self, targetAlignmentMode: TargetAlignmentMode, targetAlignmentLocation: TargetAlignmentLocation) -> Command:
    return self._robot.drive.alignToTarget(
      self._robot.localization.getRobotPose, 
      lambda: self._robot.localization.getTargetPose(targetAlignmentLocation, self._robot.elevator.isAtReefCoralL4Position()),
      targetAlignmentMode
    ).until(
      lambda: self.isRobotAlignedToTarget()
    ).withTimeout(
      constants.Game.Commands.kTargetAlignmentTimeout
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Both)
    ).withName(f'Game:AlignRobotToTarget:{ targetAlignmentMode.name }:{ targetAlignmentLocation.name }')
  
  def isRobotAlignedToTarget(self) -> bool:
    return self._robot.drive.isAlignedToTarget()

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
        TargetPositionType.FunnelIntake: self._alignRobotToTargetPositionFunnelIntake(),
        TargetPositionType.IntakeReady: self._alignRobotToTargetPosition(TargetPositionType.IntakeReady),
        TargetPositionType.Intake: self._alignRobotToTargetPosition(TargetPositionType.Intake),
        TargetPositionType.IntakeLift: self._alignRobotToTargetPosition(TargetPositionType.IntakeLift),
        TargetPositionType.CageDeepClimb: self._alignRobotToTargetPositionCageDeepClimb()
      }, 
      lambda: targetPositionType
    ).withName(f'Game:AlignRobotToTargetPosition:{ targetPositionType.name }')
  
  def _alignRobotToTargetPosition(self, targetPositionType: TargetPositionType):
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
      self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm),
      self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist),
      self._robot.hand.runGripper().unless(
        lambda: targetPositionType in [ TargetPositionType.ReefAlgaeL3, TargetPositionType.ReefAlgaeL2, TargetPositionType.IntakeReady ]
      ),
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Driver)),
      cmd.runOnce(lambda: self._setIsFunnelReady(False))
    )

  def _alignRobotToTargetPositionCoralStation(self):
    return cmd.parallel(
      cmd.waitUntil(lambda: self._robot.arm.isAtCoralStationSafePosition()).andThen(self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].elevator, isParallel = False)),
      self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].arm),
      cmd.waitUntil(lambda: self._robot.arm.isAtCoralStationSafePosition()).andThen(self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].wrist)),
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self._robot.hand.runGripper()),
      cmd.waitUntil(lambda: self.isGripperHolding()).andThen(self.rumbleControllers(ControllerRumbleMode.Both)),
      cmd.runOnce(lambda: self._setIsFunnelReady(False))
    )

  def _alignRobotToTargetPositionFunnelIntake(self):
    return cmd.either(
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelReady].elevator, isParallel = False),
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelReady].arm),
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelReady].wrist),
        cmd.runOnce(lambda: self._setIsFunnelReady(True))
      ),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelIntake].elevator),
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelIntake].arm),
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelIntake].wrist),
        self._robot.hand.runGripper(),
        cmd.runOnce(lambda: self._setIsFunnelReady(False))
      ).until(
        lambda: self.isGripperHolding()
      ).andThen(
        cmd.parallel(
          self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelLift].elevator),
          self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelLift].arm),
          self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.FunnelLift].wrist),
          self._robot.hand.runGripper(),
          self.rumbleControllers(ControllerRumbleMode.Both)
        )
      ),
      lambda: not self._isFunnelReady
    )
  
  def _setIsFunnelReady(self, isFunnelReady: bool) -> None:
    self._isFunnelReady = isFunnelReady
  
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
  
  def intake(self) -> Command:
    return cmd.parallel(
      self._robot.intake.intake(),
      self.alignRobotToTargetPosition(TargetPositionType.IntakeReady)
    )
  
  def moveCoralToGripper(self) -> Command:
    return cmd.sequence(
      self._robot.intake.alignToPosition(constants.Subsystems.Intake.kTransitionPosition),
      self.alignRobotToTargetPosition(TargetPositionType.Intake).until(lambda: self.isGripperHolding),
      self.alignRobotToTargetPosition(TargetPositionType.IntakeLift)
    )

  def isRobotAlignedToTargetPosition(self) -> bool:
    return (
      self._robot.elevator.isAlignedToPosition() and 
      self._robot.arm.isAlignedToPosition() and 
      self._robot.wrist.isAlignedToPosition()
    )

  def intakeGripper(self) -> Command:
    return self._robot.hand.runGripper(
      isManual = True
    ).alongWith(
      self._robot.wrist.refreshPosition()
    ).withName("Game:IntakeManual")
  
  def isGripperHolding(self) -> bool:
    return self._robot.hand.isGripperHolding()

  def score(self) -> Command:
    return cmd.either(
      self._robot.hand.releaseGripper(isLowSpeed = True),
      self._robot.hand.releaseGripper(isLowSpeed = False),
      lambda: self._robot.arm.isAtReefCoralL1Position()
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Driver)
    ).withName("Game:Score")
  
  def isRobotAlignedForScoring(self) -> bool:
    return (
      self.isGripperHolding() and
      self.isRobotAlignedToTargetPosition() and
      self.isRobotAlignedToTarget()
    )
  
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

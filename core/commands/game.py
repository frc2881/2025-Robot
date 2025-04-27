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
      lambda: self._robot.localization.getTargetPose(targetAlignmentLocation, self._robot.elevator.isAtReefCoralL4Position()),
      targetAlignmentMode
    ).until(
      lambda: self.isRobotAlignedToTarget()
    ).withTimeout(
      constants.Game.Commands.kTargetAlignmentTimeout
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Driver)
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
        TargetPositionType.IntakeReady: self._alignRobotToTargetPosition(TargetPositionType.IntakeReady),
        TargetPositionType.IntakeHandoff: self._alignRobotToTargetPosition(TargetPositionType.IntakeHandoff),
        TargetPositionType.IntakeLift: self._alignRobotToTargetPosition(TargetPositionType.IntakeLift),
        TargetPositionType.CageIntercept: self._alignRobotToTargetPositionCageIntercept(),
        TargetPositionType.CageDeepClimb: self._alignRobotToTargetPositionCageDeepClimb()
      }, 
      lambda: targetPositionType
    ).withName(f'Game:AlignRobotToTargetPosition:{ targetPositionType.name }')
  
  def isRobotAlignedToTargetPosition(self) -> bool:
    return (
      self._robot.elevator.isAlignedToPosition() and 
      self._robot.arm.isAlignedToPosition() and 
      self._robot.wrist.isAlignedToPosition()
    )

  def _alignRobotToTargetPosition(self, targetPositionType: TargetPositionType):
    return cmd.parallel(
      self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].elevator),
      self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].arm),
      self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[targetPositionType].wrist),
      self._robot.hand.runGripper().unless(lambda: targetPositionType in [ TargetPositionType.IntakeReady ]),
      self._robot.intake.hold(),
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
    )

  def _alignRobotToTargetPositionCoralStation(self):
    return cmd.parallel(
      cmd.waitUntil(lambda: self._robot.arm.isAtCoralStationSafePosition()).andThen(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].elevator, isParallel = False)
      ),
      self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].arm),
      cmd.waitUntil(lambda: self._robot.arm.isAtCoralStationSafePosition()).andThen(
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CoralStation].wrist)
      ),
      self._robot.hand.runGripper(),
      cmd.waitUntil(lambda: self.isGripperHolding()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
    )
  
  def _alignRobotToTargetPositionCageIntercept(self) -> Command:
    return cmd.sequence(
      cmd.waitSeconds(0.5),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageIntercept].elevator, isParallel = False),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageIntercept].arm)
        ),
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageIntercept].wrist),
        self._robot.intake.climb()
      )
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
    )

  def _alignRobotToTargetPositionCageDeepClimb(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(lambda: self._robot.elevator.setUpperStageSoftLimitsEnabled(False)),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].elevator, isParallel = False),
        cmd.waitUntil(lambda: self._robot.elevator.isAlignedToPosition()).andThen(
          self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].arm)
        ),
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.CageDeepClimb].wrist),
        cmd.waitUntil(lambda: self._robot.arm.isAlignedToPosition()).andThen(
          self._robot.intake.climb()
        )
      )
    ).alongWith(
      cmd.waitUntil(lambda: self.isRobotAlignedToTargetPosition()).andThen(self.rumbleControllers(ControllerRumbleMode.Both))
    ).finallyDo(
      lambda end: self._robot.elevator.setUpperStageSoftLimitsEnabled(True)
    )
    
  def intakeCoral(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].elevator),
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].wrist),
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeReady].arm),
        self._robot.intake.intake()
      ).until(lambda: self.isIntakeHolding()),
      self.liftCoral()
    ).onlyIf(
      lambda: not self.isGripperHolding()
    ).withName("Game:IntakeCoral")

  def liftCoral(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeHandoff].elevator),
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeHandoff].arm),
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeHandoff].wrist),
        cmd.sequence(
          self._robot.intake.alignToPosition(constants.Subsystems.Intake.kHandoffPosition).until(lambda: self.isRobotAlignedToTargetPosition()),
          cmd.parallel(
            self._robot.hand.runGripper(),
            self._robot.intake.handoff(),
            self.rumbleControllers(ControllerRumbleMode.Both)
          )
        ),
      ).until(lambda: self.isGripperHolding()),
      cmd.parallel(
        self._robot.elevator.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeLift].elevator, isParallel = False),
        self._robot.arm.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeLift].arm),
        self._robot.wrist.alignToPosition(constants.Game.Field.Targets.kTargetPositions[TargetPositionType.IntakeLift].wrist),
        self._robot.hand.runGripper()
      )
    ).onlyIf(
      lambda: self.isIntakeHolding()
    ).withName("Game:LiftCoral")

  def scoreCoral(self) -> Command:
    return cmd.either(
      self._robot.hand.releaseGripper(isLowSpeed = True),
      self._robot.hand.releaseGripper(isLowSpeed = False),
      lambda: self._robot.arm.isAtReefCoralL1Position()
    ).andThen(
      self.rumbleControllers(ControllerRumbleMode.Both)
    ).withName("Game:ScoreCoral")

  def runGripper(self) -> Command:
    return self._robot.hand.runGripper(
      isManual = True
    ).alongWith(
      self._robot.wrist.refreshPosition()
    ).withName("Game:RunGripper")
  
  def isGripperHolding(self) -> bool:
    return self._robot.hand.isGripperHolding()
  
  def isIntakeHolding(self) -> bool:
    return self._robot.intakeSensor.hasTarget()
  
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

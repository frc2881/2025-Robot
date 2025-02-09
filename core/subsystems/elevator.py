from typing import Callable
import math
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
from core.classes import ElevatorPositions, ReefLevel
import core.constants as constants

class ElevatorSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Elevator

    self._hasInitialZeroResetLowerStage: bool = False
    self._hasInitialZeroResetUpperStage: bool = False
    self._isAlignedToPositions: bool = False

    self._lowerStageModule = PositionControlModule(self._constants.kLowerStageModuleConfig)
    self._upperStageModule = PositionControlModule(self._constants.kUpperStageModuleConfig)

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runCommand(self, getInput: Callable[[], units.percent], getArmPosition: Callable[[], float]) -> Command:
    return self.run(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.clearPositionsAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("ElevatorSubsystem:Run")

  def alignToPositionsCommand(self, elevatorPositions: ElevatorPositions) -> Command:
    return self.run(
      lambda: [
        self._setPositions(elevatorPositions),
        self._setIsAlignedToPositions(elevatorPositions)
      ]
    ).beforeStarting(
      lambda: self.clearPositionsAlignment()
    ).withName("ElevatorSubsystem:AlignToPositions")

  def _setSpeed(self, speed: units.percent) -> None:
    isUpperStageAtSoftLimit = self._upperStageModule.isReverseSoftLimitReached if speed < 0 else self._upperStageModule.isForwardSoftLimitReached
    self._lowerStageModule.setSpeed(speed if isUpperStageAtSoftLimit else 0)
    self._upperStageModule.setSpeed(speed if not isUpperStageAtSoftLimit else 0)

  def _setPositions(self, elevatorPositions: ElevatorPositions) -> None:
    self._lowerStageModule.setPosition(elevatorPositions.lowerStage)
    self._upperStageModule.setPosition(elevatorPositions.upperStage)

  def _setIsAlignedToPositions(self, elevatorPositions: ElevatorPositions) -> None:
    self._isAlignedToPositions = (
      (math.fabs(self._lowerStageModule.getPosition() - elevatorPositions.lowerStage) <= self._constants.kPositionsAlignmentPositionTolerance) 
      and 
      (math.fabs(self._upperStageModule.getPosition() - elevatorPositions.upperStage) <= self._constants.kPositionsAlignmentPositionTolerance)
    )
  
  def getPositions(self) -> ElevatorPositions:
    return ElevatorPositions(self._lowerStageModule.getPosition(), self._upperStageModule.getPosition())

  def isAlignedToPositions(self) -> bool:
    return self._isAlignedToPositions
  
  def clearPositionsAlignment(self) -> None:
    self._isAlignedToPositions = False

  def resetToZeroLowerStageCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._lowerStageModule.startZeroReset()
      ],
      lambda: [
        self._lowerStageModule.endZeroReset(),
        setattr(self, "_hasInitialZeroResetLowerStage", True)
      ]
    ).withName("ElevatorSubsystem:ResetToZeroLower")

  def resetToZeroUpperStageCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._upperStageModule.startZeroReset()
      ],
      lambda: [
        self._upperStageModule.endZeroReset(),
        setattr(self, "_hasInitialZeroResetUpperStage", True)
      ]
    ).withName("ElevatorSubsystem:ResetToZeroUpper")
  
  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroResetLowerStage and self._hasInitialZeroResetUpperStage
  
  def reset(self) -> None:
    self._lowerStageModule.reset()
    self._upperStageModule.reset()
    self.clearPositionsAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Elevator/LowerStage/Position", self._lowerStageModule.getPosition())
    SmartDashboard.putNumber("Robot/Elevator/UpperStage/Position", self._upperStageModule.getPosition())
    SmartDashboard.putBoolean("Robot/Elevator/IsAlignedToPositions", self._isAlignedToPositions)
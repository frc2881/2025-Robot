from typing import Callable
import math
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.classes import MotorDirection
from lib.components.position_control_module import PositionControlModule
from core.classes import ElevatorPosition
import core.constants as constants

class ElevatorSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Elevator

    self._hasInitialZeroResetLowerStage: bool = False
    self._hasInitialZeroResetUpperStage: bool = False
    self._isAlignedToPosition: bool = False

    self._lowerStageModule = PositionControlModule(self._constants.kLowerStageModuleConfig)
    self._upperStageModule = PositionControlModule(self._constants.kUpperStageModuleConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.resetPositionAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("ElevatorSubsystem:Run")

  def alignToPositionCommand(self, elevatorPosition: ElevatorPosition) -> Command:
    return self.run(
      lambda: [
        self._setPosition(elevatorPosition),
        self._setIsAlignedToPosition(elevatorPosition)
      ]
    ).beforeStarting(
      lambda: self.resetPositionAlignment()
    ).withName("ElevatorSubsystem:AlignToPosition")

  def _setSpeed(self, speed: units.percent) -> None:
    self._upperStageModule.setSpeed(speed) 
    self._lowerStageModule.setSpeed(speed if self._upperStageModule.isPositionAtSoftLimit(MotorDirection.Forward if speed > 0 else MotorDirection.Reverse, 1.0) else 0)

  def _setPosition(self, elevatorPosition: ElevatorPosition) -> None:
    self._upperStageModule.setPosition(elevatorPosition.upperStage)
    self._lowerStageModule.setPosition(elevatorPosition.lowerStage)

  def _setIsAlignedToPosition(self, elevatorPosition: ElevatorPosition) -> None:
    self._isAlignedToPosition = (
      math.isclose(self._lowerStageModule.getPosition(), elevatorPosition.lowerStage, abs_tol = self._constants.kPositionAlignmentPositionTolerance)
      and 
      math.isclose(self._upperStageModule.getPosition(), elevatorPosition.upperStage, abs_tol = self._constants.kPositionAlignmentPositionTolerance)
    )

  def getPosition(self) -> ElevatorPosition:
    return ElevatorPosition(self._lowerStageModule.getPosition(), self._upperStageModule.getPosition())

  def isAlignedToPosition(self) -> bool:
    return self._isAlignedToPosition
  
  def resetPositionAlignment(self) -> None:
    self._isAlignedToPosition = False

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
    self.resetPositionAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Elevator/IsAlignedToPosition", self._isAlignedToPosition)
    SmartDashboard.putNumber("Robot/Elevator/Position/LowerStage", self._lowerStageModule.getPosition())
    SmartDashboard.putNumber("Robot/Elevator/Position/UpperStage", self._upperStageModule.getPosition())

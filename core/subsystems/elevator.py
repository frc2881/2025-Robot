from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.classes import MotorDirection
from lib.components.position_control_module import PositionControlModule
from core.classes import ElevatorPosition, ElevatorStage
import core.constants as constants

class Elevator(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Elevator

    self._hasInitialZeroReset: bool = False

    self._lowerStage = PositionControlModule(self._constants.kLowerStageConfig)
    self._upperStage = PositionControlModule(self._constants.kUpperStageConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def default(self, getInput: Callable[[], units.percent], elevatorStage: ElevatorStage = ElevatorStage.Both) -> Command:
    return self.runEnd(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit, elevatorStage),
      lambda: self.reset()
    ).withName(f'Elevator:Run:{ elevatorStage.name }')
  
  def _setSpeed(self, speed: units.percent, elevatorStage: ElevatorStage) -> None:
    match elevatorStage:
      case ElevatorStage.Lower:
        self._lowerStage.setSpeed(speed)
        self._upperStage.setSpeed(0)
      case ElevatorStage.Upper:
        self._lowerStage.setSpeed(0)
        self._upperStage.setSpeed(speed)
      case ElevatorStage.Both:
        self._lowerStage.setSpeed(
          speed 
          if elevatorStage == ElevatorStage.Lower 
          or self._upperStage.isAtSoftLimit(MotorDirection.Forward if speed > 0 else MotorDirection.Reverse, self._constants.kUpperStageSoftLimitBuffer) 
          else 0
        )
        self._upperStage.setSpeed(speed)
  
  def alignToPosition(self, elevatorPosition: ElevatorPosition) -> Command:
    return self.run(
      lambda: [
        self._lowerStage.alignToPosition(elevatorPosition.lowerStage),
        self._upperStage.alignToPosition(elevatorPosition.upperStage)
      ]
    ).withName("Elevator:AlignToPosition")

  def getPosition(self) -> ElevatorPosition:
    return ElevatorPosition(self._lowerStage.getPosition(), self._upperStage.getPosition())

  def isAlignedToPosition(self) -> bool:
    return self._lowerStage.isAlignedToPosition() and self._upperStage.isAlignedToPosition()
  
  def isReefCoralL4Ready(self) -> bool:
    return self.getPosition().lowerStage > self._constants.kLowerStageReefCoralL4ReadyPosition
  
  def resetLowerStageToZero(self) -> Command:
    return self._lowerStage.resetToZero(self).withName("Elevator:ResetLowerStageToZero")

  def resetUpperStageToZero(self) -> Command:
    return self._upperStage.resetToZero(self).withName("Elevator:ResetUpperStageToZero")
  
  def hasZeroReset(self) -> bool:
    return self._lowerStage.hasZeroReset() and self._upperStage.hasZeroReset()
  
  def reset(self) -> None:
    self._lowerStage.reset()
    self._upperStage.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Elevator/IsAlignedToPosition", self.isAlignedToPosition())

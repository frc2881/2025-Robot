from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.classes import MotorDirection
from lib.components.position_control_module import PositionControlModule
from core.classes import ElevatorPosition, ElevatorStage
import core.constants as constants

class ElevatorSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Elevator

    self._hasInitialZeroReset: bool = False

    self._lowerStage = PositionControlModule(self._constants.kLowerStageConfig)
    self._upperStage = PositionControlModule(self._constants.kUpperStageConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, getInput: Callable[[], units.percent], elevatorStage: ElevatorStage = ElevatorStage.Both) -> Command:
    return self.runEnd(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit, elevatorStage),
      lambda: self.reset()
    ).withName("ElevatorSubsystem:Run")
  
  def _setSpeed(self, speed: units.percent, elevatorStage: ElevatorStage) -> None:
    if elevatorStage != elevatorStage.Upper:
      self._lowerStage.setSpeed(
        speed 
        if elevatorStage == ElevatorStage.Lower 
        or self._upperStage.isAtSoftLimit(MotorDirection.Forward if speed > 0 else MotorDirection.Reverse, self._constants.kUpperStageSoftLimitBuffer) 
        else 0
      )
    if elevatorStage != elevatorStage.Lower:
      self._upperStage.setSpeed(speed)

  def alignToPositionCommand(self, elevatorPosition: ElevatorPosition) -> Command:
    return self.run(
      lambda: [
        self._lowerStage.alignToPosition(elevatorPosition.lowerStage),
        self._upperStage.alignToPosition(elevatorPosition.upperStage)
      ]
    ).withName("ElevatorSubsystem:AlignToPosition")

  def setPosition(self, elevatorPosition: ElevatorPosition) -> Command:
    return self.run(
      lambda: [
        self._lowerStage.setPosition(elevatorPosition.lowerStage),
        self._upperStage.setPosition(elevatorPosition.upperStage)
      ]
    ).withName("ElevatorSubsystem:SetPosition")

  def getPosition(self) -> ElevatorPosition:
    return ElevatorPosition(self._lowerStage.getPosition(), self._upperStage.getPosition())

  def isAlignedToPosition(self) -> bool:
    return self._lowerStage.isAlignedToPosition() and self._upperStage.isAlignedToPosition()
  
  def suspendSoftLimitsCommand(self) -> Command:
    return self._lowerStage.suspendSoftLimitsCommand().alongWith(self._upperStage.suspendSoftLimitsCommand()).withName("ElevatorSubsystem:SuspendSoftLimitsCommand")

  def resetLowerStageToZeroCommand(self) -> Command:
    return self._lowerStage.resetToZeroCommand(self).withName("ElevatorSubsystem:ResetLowerStageToZero")

  def resetUpperStageToZeroCommand(self) -> Command:
    return self._upperStage.resetToZeroCommand(self).withName("ElevatorSubsystem:ResetUpperStageToZero")
  
  def hasInitialZeroReset(self) -> bool:
    return self._lowerStage.hasInitialZeroReset() and self._upperStage.hasInitialZeroReset()
  
  def reset(self) -> None:
    self._lowerStage.reset()
    self._upperStage.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Elevator/IsAlignedToPosition", self.isAlignedToPosition())

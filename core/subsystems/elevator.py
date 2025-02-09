import math
from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
from core.classes import ElevatorStagePositions, ReefLevel
import core.constants as constants

class ElevatorSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Elevator

    self._hasInitialZeroResetUpper: bool = False
    self._hasInitialZeroResetLower: bool = False
    self._isAlignedToHeight: bool = False

    self._leadscrewModuleLower = PositionControlModule(self._constants.kLeadScrewModuleConfigLower)
    self._leadscrewModuleUpper = PositionControlModule(self._constants.kLeadScrewModuleConfigUpper)

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.clearHeightAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("ElevatorSubsystem:Run")

  def alignToHeightCommand(self, elevatorPosition: ElevatorStagePositions) -> Command:
    return self.run(
      lambda: [
        self._setPosition(elevatorPosition),
        self._setIsAlignedToHeight(elevatorPosition)
      ]
    ).beforeStarting(
      lambda: self.clearHeightAlignment()
    ).withName("ElevatorSubsystem:AlignToPosition")

  def _setSpeed(self, speed: units.percent) -> None:
    # TODO: check this logic to see about optimization of travel and remove reaching into private module constants
    if math.fabs(self._leadscrewModuleUpper.getPosition() - (self._leadscrewModuleUpper._config.constants.motorSoftLimitReverse if speed < 0 else self._leadscrewModuleUpper._config.constants.motorSoftLimitForward)) < 0.1:
      self._leadscrewModuleLower.setSpeed(speed)
      self._leadscrewModuleUpper.setSpeed(0)
    else:
      self._leadscrewModuleLower.setSpeed(0) 
      self._leadscrewModuleUpper.setSpeed(speed)

  def _setPosition(self, elevatorStagePositions: ElevatorStagePositions) -> None:
    self._leadscrewModuleLower.setPosition(elevatorStagePositions.lower)
    self._leadscrewModuleUpper.setPosition(elevatorStagePositions.upper)

  def _getIndividualPositions(self, height: units.meters) -> ElevatorStagePositions:
    # TODO: Determine whether it's more optimal to have set upper/lower heights or calculate the correct positions based on overall height
    pass

  def _getHeight(self) -> ElevatorStagePositions:
    return ElevatorStagePositions(self._leadscrewModuleLower.getPosition(), self._leadscrewModuleUpper.getPosition())

  def _setIsAlignedToHeight(self, elevatorPosition: ElevatorStagePositions) -> None:
    self._isAlignedToHeight = (math.fabs(self._getHeight().lower - elevatorPosition.lower) <= self._constants.kHeightAlignmentPositionTolerance) and (math.fabs(self._getHeight().upper - elevatorPosition.upper) <= self._constants.kHeightAlignmentPositionTolerance)

  def isAlignedToHeight(self) -> bool:
    return self._isAlignedToHeight
  
  def clearHeightAlignment(self) -> None:
    self._isAlignedToHeight = False

  def resetToZeroCommandUpper(self) -> Command:
    return self.startEnd(
      lambda: [
        self._leadscrewModuleUpper.startZeroReset()
      ],
      lambda: [
        self._leadscrewModuleUpper.endZeroReset(),
        setattr(self, "_hasInitialZeroResetUpper", True)
      ]
    ).withName("ElevatorSubsystem:ResetToZeroUpper")
  
  def resetToZeroCommandLower(self) -> Command:
    return self.startEnd(
      lambda: [
        self._leadscrewModuleLower.startZeroReset()
      ],
      lambda: [
        self._leadscrewModuleLower.endZeroReset(),
        setattr(self, "_hasInitialZeroResetLower", True)
      ]
    ).withName("ElevatorSubsystem:ResetToZeroLower")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroResetUpper and self._hasInitialZeroResetLower
  

  def reset(self) -> None:
    self._leadscrewModuleLower.reset()
    self._leadscrewModuleUpper.reset()
    self.clearHeightAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Elevator/Lower/Position", self._leadscrewModuleLower.getPosition())
    SmartDashboard.putNumber("Robot/Elevator/Upper/Position", self._leadscrewModuleUpper.getPosition())
    SmartDashboard.putBoolean("Robot/Elevator/IsAlignedToHeight", self._isAlignedToHeight)
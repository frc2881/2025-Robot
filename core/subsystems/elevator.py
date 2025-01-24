import math
from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
from lib.components.leadscrew_module import LeadscrewModule
from core.classes import ElevatorStagePositions
import core.constants as constants

class ElevatorSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Elevator

    self._hasInitialZeroReset: bool = False
    self._isAlignedToHeight: bool = False

    self._leadscrewModuleLower = LeadscrewModule(self._constants.kLeadScrewModuleConfigLower)
    self._leadscrewModuleUpper = LeadscrewModule(self._constants.kLeadScrewModuleConfigUpper)
    
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

  def alignToHeightCommand(self, height: units.meters) -> Command:
    return self.run(
      lambda: [
        self._setPosition(self._getIndividualPositions(height)),
        self._setIsAlignedToHeight(height)
      ]
    ).beforeStarting(
      lambda: self.clearHeightAlignment()
    ).withName("ElevatorSubsystem:AlignToPosition")

  def _setSpeed(self, speed: units.percent) -> None:
    self._leadscrewModuleLower.setSpeed(speed)
    self._leadscrewModuleUpper.setSpeed(speed)

  def _setPosition(self, elevatorStagePositions: ElevatorStagePositions) -> None:
    self._leadscrewModuleLower.setPosition(elevatorStagePositions.lower)
    self._leadscrewModuleUpper.setPosition(elevatorStagePositions.upper)

  def _getIndividualPositions(self, height: units.meters) -> ElevatorStagePositions:
    # TODO: calculate the individual elevator position?
    pass

  def _getHeight(self) -> float:
    return self._leadscrewModuleLower.getPosition() + self._leadscrewModuleUpper.getPosition()

  def _setIsAlignedToHeight(self, height: float) -> None:
    self._isAlignedToHeight = math.fabs(self._getHeight() - height) <= self._constants.kHeightAlignmentPositionTolerance

  def isAlignedToHeight(self) -> bool:
    return self._isAlignedToHeight
  
  def clearHeightAlignment(self) -> None:
    self._isAlignedToHeight = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._leadscrewModuleLower.startZeroReset(),
        self._leadscrewModuleUpper.startZeroReset()
      ],
      lambda: [
        self._leadscrewModuleLower.endZeroReset(),
        self._leadscrewModuleUpper.endZeroReset(),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("ElevatorSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._leadscrewModuleLower.reset()
    self._leadscrewModuleUpper.reset()
    self.clearHeightAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Elevator/Lower/Position", self._leadscrewModuleLower.getPosition())
    SmartDashboard.putNumber("Robot/Elevator/Upper/Position", self._leadscrewModuleUpper.getPosition())
    SmartDashboard.putBoolean("Robot/Elevator/IsAlignedToHeight", self._isAlignedToHeight)
from typing import Callable
import math
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
import core.constants as constants

class ArmSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Arm

    self._hasInitialZeroReset: bool = False
    self._isAlignedToPosition: bool = False

    self._armModule = PositionControlModule(self._constants.kArmPositonControlModuleConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._setSpeed(-getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.resetPositionAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("ArmSubsystem:Run")
  
  def alignToPositionCommand(self, position: units.inches) -> Command:
    return self.run(
      lambda: [
        self._setPosition(position),
        self._setIsAlignedToPosition(position)
      ]
    ).beforeStarting(
      lambda: self.resetPositionAlignment()
    ).withName("ArmSubsystem:AlignToPosition")
  
  def _setSpeed(self, speed: units.percent) -> None:
    self._armModule.setSpeed(speed)

  def _setPosition(self, position: units.inches) -> None:
    self._armModule.setPosition(position)

  def getPosition(self) -> units.inches:
    return self._armModule.getPosition()

  def _setIsAlignedToPosition(self, position: units.inches) -> None:
    self._isAlignedToPosition = math.isclose(self.getPosition(), position, abs_tol = self._constants.kPositionAlignmentPositionTolerance)
  
  def isAlignedToPosition(self) -> bool:
    return self._isAlignedToPosition
  
  def resetPositionAlignment(self) -> None:
    self._isAlignedToPosition = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._armModule.startZeroReset(),
      ],
      lambda: [
        self._armModule.endZeroReset(),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("ArmSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._armModule.reset()
    self.resetPositionAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Arm/IsAlignedToPosition", self._isAlignedToPosition)
    SmartDashboard.putNumber("Robot/Arm/Position", self.getPosition())

from typing import Callable
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

    self._arm = PositionControlModule(self._constants.kArmConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.runEnd(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit),
      lambda: self.reset()
    ).withName("ArmSubsystem:Run")
  
  def alignToPositionCommand(self, position: units.inches) -> Command:
    return self.run(
      lambda: self._setPosition(position)
    ).withName("ArmSubsystem:AlignToPosition")
  
  def _setSpeed(self, speed: units.percent) -> None:
    self._arm.setSpeed(speed)

  def _setPosition(self, position: units.inches) -> None:
    self._arm.setPosition(position)

  def getPosition(self) -> units.inches:
    return self._arm.getPosition()

  def isAlignedToPosition(self) -> bool:
    return self._arm.isAlignedToPosition()
  
  def resetPositionAlignment(self) -> None:
    self._arm.resetPositionAlignment()
  
  def resetToZeroCommand(self) -> Command:
    return self._arm.resetToZeroCommand(self).withName("ArmSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._arm.hasInitialZeroReset()

  def reset(self) -> None:
    self._arm.reset()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Arm/IsAlignedToPosition", self.isAlignedToPosition())

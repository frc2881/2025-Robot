import math
from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig, ClosedLoopConfig
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
import core.constants as constants

class ArmSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Arm

    self._hasInitialZeroReset: bool = False
    self._isAlignedToTarget: bool = False

    self._armMotor = PositionControlModule(self._constants.kArmPositonControlModuleConfig)

  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("ArmSubsystem:Run")
  
  def alignToPositionCommand(self, position: float) -> Command:
    return self.run(
      lambda: [
        self._setPosition(position),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).withName("ArmSubsystem:AlignToPosition")
  
  def _setSpeed(self, speed: units.percent) -> None:
    self._armMotor.setSpeed(speed)

  def _setPosition(self, position: float) -> None:
    self._armMotor.setPosition(position)

  def _getPosition(self) -> float:
    return self._armMotor.getPosition()

  def _setIsAlignedToTarget(self, position: float) -> None:
    self._isAlignedToTarget = math.fabs(self._getPosition() - position) <= self._constants.kPositionAlignmentPositionTolerance

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._armMotor.startZeroReset(),
      ],
      lambda: [
        self._armMotor.endZeroReset(),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("ArmSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._armMotor.reset()
    self.clearTargetAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Arm/Position", self._getPosition())
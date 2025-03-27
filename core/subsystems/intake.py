from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard
from wpimath import units
from rev import SparkMax, SparkBaseConfig, SparkBase
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
import core.constants as constants

class Intake(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._hasInitialZeroReset: bool = False

    self._intake = PositionControlModule(self._constants.kIntakeConfig)
    self._rollers = SparkMax(self._constants.kRollerMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kRollerMotorCurrentLimit)
      .inverted(False))
    utils.setSparkConfig(
      self._rollers.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def alignToPosition(self, position: units.inches) -> Command:
    return self.run(
      lambda: self._intake.alignToPosition(position)
    ).withName("Intake:AlignToPosition")
  
  def runRollers(self) -> Command:
    return self.runEnd(
      lambda: self._rollers.set(
        self._constants.kRollersMotorIntakeSpeed
      ),
      lambda: self._resetRollers()
    ).withName("Hand:RunRoller")
  
  def intake(self) -> Command:
    return self.runEnd(
      lambda: [
        self._intake.alignToPosition(self._constants.kIntakePosition),
        self._rollers.set(self._constants.kRollersMotorIntakeSpeed)
      ],
      lambda: self._resetRollers()
    ).withName("Intake:Intake")
  
  def eject(self) -> Command:
    return self.runEnd(
      lambda: self._rollers.set(
        self._constants.kRollersMotorEjectSpeed
      ),
      lambda: self._resetRollers()
    ).withName("Hand:RunRoller")

  def getPosition(self) -> units.inches:
    return self._intake.getPosition()

  def isAlignedToPosition(self) -> bool:
    return self._intake.isAlignedToPosition()
  
  def resetToZero(self) -> Command:
    return self._intake.resetToZero(self).withName("Intake:ResetToZero")

  def hasZeroReset(self) -> bool:
    return self._intake.hasZeroReset()

  def resetIntake(self) -> None:
    self._intake.reset()

  def _resetRollers(self) -> None:
    self._rollers.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Intake/IsAlignedToPosition", self.isAlignedToPosition())
    SmartDashboard.putNumber("Robot/Hand/Rollers/Current", self._rollers.getOutputCurrent())
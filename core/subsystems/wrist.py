from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class WristSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Wrist

    self._wristMotor = SparkMax(self._constants.kWristMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
     .smartCurrentLimit(self._constants.kWristMotorCurrentLimit)
     .inverted(True))
    utils.setSparkConfig(
      self._wristMotor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    
  def periodic(self) -> None:
    self._updateTelemetry()

  def runCommand(self, speed: units.percent) -> Command:
    return self.run(
      lambda: self._wristMotor.set(speed * self._constants.kWristInputLimit)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("WristSubsystem:Run")

  def moveUpCommand(self) -> Command:
    return self.run(
      lambda: self._wristMotor.set(self._constants.kWristMoveSpeed)
    ).finallyDo(
      lambda end: self.reset()
    ).until(
      lambda: self._wristMotor.getOutputCurrent() > self._constants.kWristMaxCurrent
    ).withName("WristSubsystem:MoveUp")
  
  def moveDownCommand(self) -> Command:
    return self.run(
      lambda: self._wristMotor.set(-self._constants.kWristMoveSpeed)
    ).finallyDo(
      lambda end: self.reset()
    ).until(
      lambda: self._wristMotor.getOutputCurrent() > self._constants.kWristMaxCurrent
    ).withName("WristSubsystem:MoveDown")

  def reset(self) -> None:
    self._wristMotor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Wrist/Current", self._wristMotor.getOutputCurrent())


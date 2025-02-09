from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command, cmd
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class WristSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Wrist

    self._isWristUp = False

    self._wristMotor = SparkMax(self._constants.kWristMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kWristMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kWristMotorCurrentLimit)
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

  def moveUpCommand(self) -> Command:
    return self.run(
      lambda: self._wristMotor.set(self._constants.kWristMoveSpeedUp)
    ).finallyDo(
      lambda end: self.reset()
    ).until(
      lambda: self._wristMotor.getOutputCurrent() > self._constants.kWristMaxCurrent
    ).withName("WristSubsystem:MoveUp")
  
  def moveDownCommand(self) -> Command:
    return self.run(
      lambda: self._wristMotor.set(-self._constants.kWristMoveSpeedDown)
    ).finallyDo(
      lambda end: self.reset()
    ).until(
      lambda: self._wristMotor.getOutputCurrent() > self._constants.kWristMaxCurrent
    ).withName("WristSubsystem:MoveDown")
  
  def toggleCommand(self) -> Command:
    return cmd.either(self.moveDownCommand(), self.moveUpCommand(), lambda: self._isWristUp)
    
  def reset(self) -> None:
    self._wristMotor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Wrist/Current", self._wristMotor.getOutputCurrent())


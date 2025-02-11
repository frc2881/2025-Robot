from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
from core.classes import WristPosition
import core.constants as constants

class WristSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Wrist

    self._position = WristPosition.Unknown
    self._isAlignedToPosition: bool = False

    self._motor = SparkMax(self._constants.kMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._motor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    
  def periodic(self) -> None:
    self._updateTelemetry()

  def setPositionCommand(self, position: WristPosition) -> Command:
    return self.run(
      lambda: self._motor.set(self._constants.kMotorUpSpeed if position != WristPosition.Up else -self._constants.kMotorDownSpeed)
    ).beforeStarting(
      lambda: self.resetPositionAlignment()
    ).until(
      lambda: self._motor.getOutputCurrent() >= self._constants.kMotorCurrentTrigger
    ).withTimeout(
      self._constants.kSetPositionTimeout
    ).finallyDo(
      lambda end: [
        self.reset(),
        setattr(self, "_position", position),
        setattr(self, "_isAlignedToPosition", True)
      ]
    ).withName("WristSubsystem:SetPosition")
  
  def togglePositionCommand(self) -> Command:
    return self.setPositionCommand(
      WristPosition.Up if self._position != WristPosition.Up else WristPosition.Down
    ).withName("WristSubsystem:TogglePosition")
    
  def getPosition(self) -> WristPosition:
    return self._position

  def isAlignedToPosition(self) -> bool:
    return self._isAlignedToPosition

  def resetPositionAlignment(self) -> None:
    self._isAlignedToPosition = False

  def reset(self) -> None:
    self._motor.stopMotor()
    self.resetPositionAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Wrist/IsAlignedToPosition", self._isAlignedToPosition)
    SmartDashboard.putString("Robot/Wrist/Position", self._position.name)
    SmartDashboard.putNumber("Robot/Wrist/Current", self._motor.getOutputCurrent())

from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import SparkMax, SparkBaseConfig, SparkBase
from lib import logger, utils
from lib.classes import Position
import core.constants as constants

class Wrist(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Wrist

    self._position = Position.Unknown
    self._isAlignedToPosition: bool = False

    self._motor = SparkMax(self._constants.kMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkBaseConfig()
    self._sparkConfig.smartCurrentLimit(self._constants.kMotorCurrentLimit)
    utils.setSparkConfig(
      self._motor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    
  def periodic(self) -> None:
    self._updateTelemetry()
    
  def alignToPosition(self, position: Position) -> Command:
    return self.setPosition(position).deadlineFor(
      cmd.waitUntil(lambda: self._position != Position.Unknown).andThen(cmd.runOnce(lambda: setattr(self, "_isAlignedToPosition", True)))
    ).beforeStarting(
      lambda: self._resetPositionAlignment()
    ).withName("Wrist:AlignToPosition")

  def setPosition(self, position: Position) -> Command:
    return self.startEnd(
      lambda: [
        self._resetPositionAlignment(),
        self._motor.set(self._constants.kMotorUpSpeed if position == Position.Up else -self._constants.kMotorDownSpeed)
      ],
      lambda: self._motor.stopMotor()
    ).withTimeout(
      self._constants.kSetPositionTimeout
    ).andThen(
      cmd.runOnce(lambda: setattr(self, "_position", position))
    ).andThen(
      self._holdPosition(position)
    ).finallyDo(
      lambda end: self._motor.stopMotor()
    ).withName("Wrist:SetPosition")

  def _holdPosition(self, position: Position) -> Command:
    return self.startEnd(
      lambda: self._motor.set(self._constants.kMotorHoldUpSpeed if position == Position.Up else -self._constants.kMotorHoldDownSpeed),
      lambda: self._motor.stopMotor()
    )
  
  def refreshPosition(self) -> Command:
    self.setPosition(self._position)

  def togglePosition(self) -> Command:
    return cmd.either(
      self.alignToPosition(Position.Up), 
      self.alignToPosition(Position.Down), 
      lambda: self._position != Position.Up
    ).withName("Wrist:TogglePosition")

  def getPosition(self) -> Position:
    return self._position
  
  def isAlignedToPosition(self) -> bool:
    return self._isAlignedToPosition

  def _resetPositionAlignment(self) -> None:
    self._position = Position.Unknown
    self._isAlignedToPosition = False

  def reset(self) -> None:
    self._motor.stopMotor()
    self._resetPositionAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Wrist/IsAlignedToPosition", self._isAlignedToPosition)
    SmartDashboard.putString("Robot/Wrist/Position", self._position.name)

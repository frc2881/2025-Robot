from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
from lib.classes import Position
import core.constants as constants

class WristSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Wrist

    self._position = Position.Unknown
    self._isAlignedToPosition: bool = False

    self._motor = SparkMax(self._constants.kMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkMaxConfig()
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

  def setPositionCommand(self, position: Position) -> Command:
    return self.startEnd(
      lambda: [
        self.resetPositionAlignment(),
        self._motor.set(
          self._constants.kMotorUpSpeed 
          if position == Position.Up else 
          -self._constants.kMotorDownSpeed)
      ],
      lambda: [
        self._motor.stopMotor(),
        self._setPosition(position)
      ]
    ).withTimeout(
      self._constants.kSetPositionTimeout
    ).withName("WristSubsystem:SetPosition")
  
  def togglePositionCommand(self) -> Command:
    return cmd.either(
      self.setPositionCommand(Position.Up), 
      self.setPositionCommand(Position.Down), 
      lambda: self._position != Position.Up
    ).withName("WristSubsystem:TogglePosition")

  def getPosition(self) -> Position:
    return self._position
  
  def _setPosition(self, position: Position) -> Command:
    self._position = position
    self._isAlignedToPosition = True

  def isAlignedToPosition(self) -> bool:
    return self._isAlignedToPosition

  def resetPositionAlignment(self) -> None:
    self._position = Position.Unknown
    self._isAlignedToPosition = False

  def reset(self) -> None:
    self._motor.stopMotor()
    self.resetPositionAlignment()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Wrist/IsAlignedToPosition", self._isAlignedToPosition)
    SmartDashboard.putString("Robot/Wrist/Position", self._position.name)
    SmartDashboard.putNumber("Robot/Wrist/Speed", self._motor.get())
    SmartDashboard.putNumber("Robot/Wrist/Current", self._motor.getOutputCurrent())

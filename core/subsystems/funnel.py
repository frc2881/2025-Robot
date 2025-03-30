from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, Servo
from lib import logger, utils
from lib.classes import Position
import core.constants as constants

class Funnel(Subsystem):
  def __init__(self) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Funnel

    self._position = Position.Closed

    self._servo = Servo(self._constants.kServoChannel)

  def periodic(self) -> None:
    self._updateTelemetry()

  def setPosition(self, position: Position) -> Command:
    return self.startEnd(
      lambda: self._servo.setPosition(
        self._constants.kPositionOpen 
        if position == Position.Open else 
        self._constants.kPositionClosed
      ),
      lambda: setattr(self, "_position", position)
    ).withTimeout(
      self._constants.kServoSetPositionTimeout
    ).withName("Funnel:SetPosition")
  
  def getPosition(self) -> Position:
    return self._position

  def reset(self) -> None:
    self._servo.setPosition(self._constants.kPositionClosed)
    self._position = Position.Closed

  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Robot/Funnel/Position", self._position.name)
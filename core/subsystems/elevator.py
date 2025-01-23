from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class ElevatorSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Elevator
    
  def periodic(self) -> None:
    self._updateTelemetry()
  
  def reset(self) -> None:
    pass

  def _updateTelemetry(self) -> None:
    # SmartDashboard.putNumber("Robot/Elevator/Speed", self._motor.get())
    pass
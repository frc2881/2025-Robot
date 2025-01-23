from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class ArmSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Arm
    
  def periodic(self) -> None:
    self._updateTelemetry()
  
  def reset(self) -> None:
    pass

  def _updateTelemetry(self) -> None:
    # SmartDashboard.putNumber("Robot/Arm/Speed", self._motor.get())
    pass
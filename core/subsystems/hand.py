from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard
from rev import SparkFlex, SparkBaseConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class Hand(Subsystem):
  def __init__(
      self, 
      gripperSensorHasTarget: Callable[[], bool]
    ) -> None:
    super().__init__()
    self._constants = constants.Subsystems.Hand

    self._gripperSensorHasTarget = gripperSensorHasTarget
    
    self._isGripperHoldingManual = False

    self._gripperMotor = SparkFlex(self._constants.kGripperMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kGripperMotorCurrentLimit)
      .inverted(False))
    utils.setSparkConfig(
      self._gripperMotor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()
      
  def runGripper(self, isManual: bool = False) -> Command:
    return self.runEnd(
      lambda: self._gripperMotor.set(
        self._constants.kGripperMotorHoldSpeed 
        if (self.isGripperHolding() and not isManual) else
        self._constants.kGripperMotorIntakeSpeed
      ),
      lambda: self._resetGripper()
    ).withName("Hand:RunGripper")
  
  def holdGripper(self) -> Command:
    return cmd.runEnd(
      lambda: setattr(self, "_isGripperHoldingManual", True),
      lambda: setattr(self, "_isGripperHoldingManual", False)
    ).withName("Hand:HoldGripper")
  
  def releaseGripper(self, isLowSpeed: bool = False) -> Command:
    return self.startEnd(
      lambda: self._gripperMotor.set(
        -self._constants.kGripperMotorReleaseSpeedLow 
        if isLowSpeed else 
        -self._constants.kGripperMotorReleaseSpeed
      ),
      lambda: self._resetGripper()
    ).withTimeout(
      self._constants.kGripperReleaseTimeout
    ).withName("Hand:ReleaseGripper")

  def isGripperEnabled(self) -> bool:
    return self._gripperMotor.get() != 0
  
  def isGripperHolding(self) -> bool:
    return self._gripperSensorHasTarget() or self._isGripperHoldingManual
  
  def _resetGripper(self) -> None:
    self._gripperMotor.stopMotor()
    self._isGripperHoldingManual = False

  def reset(self) -> None:
    self._resetGripper()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsEnabled", self.isGripperEnabled())
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsHolding", self.isGripperHolding())
    # SmartDashboard.putNumber("Robot/Hand/Gripper/Current", self._gripperMotor.getOutputCurrent())
 
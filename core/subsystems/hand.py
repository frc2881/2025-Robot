from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, PowerDistribution
from rev import SparkFlex, SparkMax, SparkBaseConfig, SparkBase
from lib import logger, utils
from lib.classes import Position
import core.constants as constants

class Hand(Subsystem):
  def __init__(self, intakeDistanceSensorHasTarget: Callable[[], bool],):
    super().__init__()
    self._constants = constants.Subsystems.Hand

    self._intakeDistanceSensorHasTarget = intakeDistanceSensorHasTarget

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

    self._suctionMotor = SparkMax(self._constants.kSuctionMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkBaseConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kSuctionMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._suctionMotor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._powerDistribution = PowerDistribution()
    self._powerDistribution.setSwitchableChannel(False)

  def periodic(self) -> None:
    self._updateTelemetry()
      
  def runGripper(self) -> Command:
    return self.run(
      lambda: self._gripperMotor.set(self._constants.kGripperMotorIntakeSpeed)
    ).until(
      lambda: self._intakeDistanceSensorHasTarget()
    ).andThen(
      self.run(
        lambda: self._gripperMotor.set(self._constants.kGripperMotorHoldSpeed)
      )
    ).finallyDo(lambda end: self._gripperMotor.stopMotor()).withName("Hand:RunGripper")
  
  def releaseGripper(self) -> Command:
    return self.startEnd(
      lambda: self._gripperMotor.set(-self._constants.kGripperMotorReleaseSpeed),
      lambda: self._resetGripper()
    ).withTimeout(
      self._constants.kGripperReleaseTimeout
    ).withName("Hand:ReleaseGripper")

  def isGripperEnabled(self) -> bool:
    return self._gripperMotor.get() != 0
  
  def isGripperHolding(self) -> bool:
    return self._intakeDistanceSensorHasTarget()
  
  def _resetGripper(self) -> None:
    self._gripperMotor.stopMotor()

  def runSuction(self) -> Command:
    return self.runOnce(
      lambda: [
        self._setSolenoidPosition(Position.Closed),
        self._suctionMotor.set(self._constants.kSuctionMotorSpeed)
      ]
    ).withName("Hand:RunSuction")

  def releaseSuction(self) -> Command:
    return self.startEnd(
      lambda: [ 
        self._suctionMotor.stopMotor(),
        self._setSolenoidPosition(Position.Open)
      ],
      lambda: self._resetSuction()
    ).withTimeout(
      self._constants.kSuctionReleaseTimeout
    ).withName("Hand:ReleaseSuction")

  def isSuctionEnabled(self) -> bool:
    return self._suctionMotor.get() != 0
  
  def isSuctionHolding(self) -> bool:
    return self._suctionMotor.getOutputCurrent() >= self._constants.kSuctionMotorCurrentTrigger
  
  def _setSolenoidPosition(self, position: Position) -> None:
    self._powerDistribution.setSwitchableChannel(True if position == Position.Open else False)

  def _resetSuction(self) -> None:
    self._suctionMotor.stopMotor()
    self._setSolenoidPosition(Position.Closed)

  def reset(self) -> None:
    self._resetGripper()
    self._resetSuction()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsEnabled", self.isGripperEnabled())
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsHolding", self.isGripperHolding())
    SmartDashboard.putNumber("Robot/Hand/Gripper/Current", self._gripperMotor.getOutputCurrent())
    SmartDashboard.putBoolean("Robot/Hand/Suction/IsEnabled", self.isSuctionEnabled())
    SmartDashboard.putBoolean("Robot/Hand/Suction/IsHolding", self.isSuctionHolding())
    SmartDashboard.putNumber("Robot/Hand/Suction/Current", self._suctionMotor.getOutputCurrent())
 
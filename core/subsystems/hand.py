from typing import Callable
from commands2 import Subsystem, Command, cmd
from wpilib import SmartDashboard, PowerDistribution
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
from lib.classes import Position
import core.constants as constants
from core.classes import TargetPositionType

class Hand(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Hand

    self._isGripperEnabled = False
    self._isGripperHolding = False
    self._isSuctionEnabled = False
    self._isSuctionHolding = False

    self._gripperMotor = SparkMax(self._constants.kGripperMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kGripperMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._gripperMotor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._suctionMotor = SparkMax(self._constants.kSuctionMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkMaxConfig()
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

  def runIntake(self, getTargetPositionType: Callable[[], TargetPositionType]):
    # TODO: Validate this
    match getTargetPositionType():
      case TargetPositionType.ReefCoralL1Score | TargetPositionType.ReefCoralL2Score | TargetPositionType.ReefCoralL3Score | TargetPositionType.ReefCoralL4Score:
        return self.runGripper()
      case TargetPositionType.ReefAlgaeL3 | TargetPositionType.ReefAlgaeL2 | TargetPositionType.AlgaeProcessor | TargetPositionType.Barge:
        return self.runSuction()
      case _:
        return cmd.none()
      
  def runGripper(self) -> Command:
    return self.startEnd(
      lambda: [
        self._gripperMotor.set(self._constants.kGripperMotorSpeed),
        setattr(self, "_isGripperEnabled", True)
      ],
      lambda: [ 
        self._gripperMotor.stopMotor(),
        setattr(self, "_isGripperEnabled", False)
      ]
    ).alongWith(
      cmd.sequence(
        cmd.runOnce(lambda: setattr(self, "_isGripperHolding", False)),
        cmd.waitSeconds(1.0),
        cmd.waitUntil(lambda: self._gripperMotor.getOutputCurrent() >= self._constants.kGripperMotorCurrentTrigger),
        cmd.runOnce(lambda: setattr(self, "_isGripperHolding", True))
      )
    ).withName("Hand:RunGripper")
  
  def releaseGripper(self) -> Command:
    return self.startEnd(
      lambda: self._gripperMotor.set(-self._constants.kGripperMotorSpeed),
      lambda: self._resetGripper()
    ).withTimeout(
      self._constants.kGripperReleaseTimeout
    ).withName("Hand:ReleaseGripper")

  def isGripperEnabled(self) -> bool:
    return self._isGripperEnabled
  
  def isGripperHolding(self) -> bool:
    return self._isGripperHolding

  def _resetGripper(self) -> None:
    self._gripperMotor.stopMotor()
    self._isGripperEnabled = False
    self._isGripperHolding = False

  def runSuction(self) -> Command:
    return self.startEnd(
      lambda: [
        self._setSolenoidPosition(Position.Closed),
        self._suctionMotor.set(self._constants.kSuctionMotorSpeed),
        setattr(self, "_isSuctionEnabled", True)
      ],
      lambda: [ 
        self._suctionMotor.stopMotor(),
        setattr(self, "_isSuctionEnabled", False)
      ]
    ).alongWith(
      cmd.sequence(
        cmd.runOnce(lambda: setattr(self, "_isSuctionHolding", False)),
        cmd.waitSeconds(2.0),
        cmd.waitUntil(lambda: self._suctionMotor.getOutputCurrent() >= self._constants.kSuctionMotorCurrentTrigger),
        cmd.runOnce(lambda: setattr(self, "_isSuctionHolding", True))
      )
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
    return self._isSuctionEnabled
  
  def isSuctionHolding(self) -> bool:
    return self._isSuctionHolding
  
  def _setSolenoidPosition(self, position: Position) -> None:
    self._powerDistribution.setSwitchableChannel(True if position == Position.Open else False)

  def _resetSuction(self) -> None:
    self._suctionMotor.stopMotor()
    self._setSolenoidPosition(Position.Closed)
    self._isSuctionEnabled = False
    self._isSuctionHolding = False

  def reset(self) -> None:
    self._resetGripper()
    self._resetSuction()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsEnabled", self._isGripperEnabled)
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsHolding", self._isGripperHolding)
    SmartDashboard.putNumber("Robot/Hand/Gripper/Current", self._gripperMotor.getOutputCurrent()) 
    SmartDashboard.putBoolean("Robot/Hand/Suction/IsEnabled", self._isSuctionEnabled)
    SmartDashboard.putBoolean("Robot/Hand/Suction/IsHolding", self._isSuctionHolding)
    SmartDashboard.putNumber("Robot/Hand/Suction/Current", self._suctionMotor.getOutputCurrent())
 
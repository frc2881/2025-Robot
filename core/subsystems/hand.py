from wpilib import SmartDashboard, PowerDistribution
from commands2 import Subsystem, Command, cmd
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class HandSubsystem(Subsystem):
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

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def runGripperCommand(self) -> Command:
    return self.run(
      lambda: [
        self._gripperMotor.set(self._constants.kGripperMotorIntakeSpeed),
        setattr(self, "_isGripperEnabled", True)
      ]
    ).withTimeout(
      0.5
    ).andThen(
      lambda: self._gripperMotor.set(self._constants.kGripperMotorIntakeSpeed)
    ).until(
      # TODO: possibly refine output current trigger logic with moving average and delta/spike detection
      lambda: self._gripperMotor.getOutputCurrent() >= self._constants.kGripperMotorCurrentTrigger
    ).andThen(
      lambda: [ 
        self._gripperMotor.set(self._constants.kGripperMotorHoldSpeed),
        setattr(self, "_isGripperHolding", True)
      ]
      # TODO: consider adding logic that detects losing the gripper hold
    ).withName("HandSubsystem:RunGripper")
  
  def releaseGripperCommand(self) -> Command:
    return self.run(
      lambda: self._gripperMotor.set(self._constants.kGripperMotorReleaseSpeed)
    ).withTimeout(
      0.5
    ).finallyDo(
      lambda end: self._resetGripper()
    ).withName("HandSubsystem:ReleaseGripper")

  def toggleGripperCommand(self) -> Command:
    return cmd.either(
      self.runGripperCommand(), 
      self.releaseGripperCommand(), 
      lambda: not self._isGripperEnabled
    ).withName("HandSubsystem:ToggleGripper")

  def runSuctionCommand(self) -> Command:
    return self.run(
      lambda: [
        self._powerDistribution.setSwitchableChannel(False),
        self._suctionMotor.set(self._constants.kSuctionMotorIntakeSpeed),
        setattr(self, "_isSuctionEnabled", True)
      ]
    ).withTimeout(
      0.5
    ).andThen(
      lambda: self._suctionMotor.set(self._constants.kSuctionMotorIntakeSpeed)
    ).until(
      # TODO: possibly refine output current trigger logic with moving average and delta/spike detection
      lambda: self._suctionMotor.getOutputCurrent() >= self._constants.kSuctionMotorCurrentTrigger
    ).andThen(
      lambda: setattr(self, "_isSuctionHolding", True)
      # TODO: consider adding logic that detects losing the suction hold
    ).withName("HandSubsystem:RunSuction")
  
  def releaseSuctionCommand(self) -> Command:
    return self.runOnce(
      lambda: self._resetSuction()
    ).withName("HandSubsystem:ReleaseSuction")

  def toggleSuctionCommand(self) -> Command:
    return cmd.either(
      self.runSuctionCommand(), 
      self.releaseSuctionCommand(), 
      lambda: not self._isSuctionEnabled
    ).withName("HandSubsystem:ToggleSuction")

  def _resetGripper(self) -> None:
    self._gripperMotor.stopMotor()
    self._isGripperHolding = False
    self._isGripperEnabled = False

  def _resetSuction(self) -> None:
    self._suctionMotor.stopMotor()
    self._powerDistribution.setSwitchableChannel(True)
    self._isSuctionHolding = False
    self._isSuctionEnabled = False

  def reset(self) -> None:
    self._resetGripper()
    self._resetSuction()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsEnabled", self._isGripperEnabled)
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsHolding", self._isGripperHolding)
    SmartDashboard.putNumber("Robot/Hand/Gripper/Speed", self._gripperMotor.get())
    SmartDashboard.putNumber("Robot/Hand/Gripper/Current", self._gripperMotor.getOutputCurrent()) 
    SmartDashboard.putBoolean("Robot/Hand/Suction/IsEnabled", self._isSuctionEnabled)
    SmartDashboard.putBoolean("Robot/Hand/Suction/IsHolding", self._isSuctionHolding)
    SmartDashboard.putNumber("Robot/Hand/Suction/Speed", self._suctionMotor.get())
    SmartDashboard.putNumber("Robot/Hand/Suction/Current", self._suctionMotor.getOutputCurrent())
 
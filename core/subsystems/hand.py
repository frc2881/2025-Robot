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
    self._isSuctionEnabled = False

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
  
  def runGripperCommand(self) -> Command:
    return self.runOnce(
      lambda: [
        self._gripperMotor.set(self._constants.kGripperMotorSpeed),
        setattr(self, "_isGripperEnabled", True)
      ]
    ).andThen(
      cmd.waitSeconds(1.0)
    ).andThen(
      cmd.startEnd(
        lambda: self._gripperMotor.set(self._constants.kGripperMotorSpeed),
        lambda: self._resetGripper()
      ).until(
        lambda: self._gripperMotor.getOutputCurrent() >= self._constants.kGripperMotorCurrentTrigger
      )
    ).withName("HandSubsystem:RunGripper")
  
  def releaseGripperCommand(self) -> Command:
    return self.startEnd(
      lambda: self._gripperMotor.set(-self._constants.kGripperMotorSpeed),
      lambda: self._resetGripper()
    ).withTimeout(1.0).withName("HandSubsystem:ReleaseGripper")

  def toggleGripperCommand(self) -> Command:
    return cmd.either(
      self.runGripperCommand(), 
      self.releaseGripperCommand(), 
      lambda: not self._isGripperEnabled
    ).withName("HandSubsystem:ToggleGripper")

  def isGripperEnabled(self) -> bool:
    return self._isGripperEnabled

  def _resetGripper(self) -> None:
    self._gripperMotor.stopMotor()
    self._isGripperEnabled = False

  def runSuctionCommand(self) -> Command:
    return self.runOnce(
      lambda: [
        self._openSolenoid(False),
        self._suctionMotor.set(self._constants.kSuctionMotorSpeed),
        setattr(self, "_isSuctionEnabled", True)
      ]
    ).andThen(
      cmd.waitSeconds(1.0)
    ).andThen(
      cmd.startEnd(
        lambda: self._suctionMotor.set(self._constants.kSuctionMotorSpeed),
        lambda: self._suctionMotor.stopMotor() # TODO: test if motor should keep running until suction is released
      ).until(
        lambda: self._suctionMotor.getOutputCurrent() >= self._constants.kSuctionMotorCurrentTrigger
      )
    ).withName("HandSubsystem:RunSuction")
  
  def releaseSuctionCommand(self) -> Command:
    return self.startEnd(
      lambda: [ 
        self._suctionMotor.stopMotor(),
        self._openSolenoid(True)
      ],
      lambda: self._resetSuction()
    ).withTimeout(1.0).withName("HandSubsystem:ReleaseSuction")

  def toggleSuctionCommand(self) -> Command:
    return cmd.either(
      self.runSuctionCommand(), 
      self.releaseSuctionCommand(), 
      lambda: not self._isSuctionEnabled
    ).withName("HandSubsystem:ToggleSuction")

  def isSuctionEnabled(self) -> bool:
    return self._isSuctionEnabled
  
  def _openSolenoid(self, isOpened: bool) -> None:
    self._powerDistribution.setSwitchableChannel(isOpened)

  def _resetSuction(self) -> None:
    self._suctionMotor.stopMotor()
    self._openSolenoid(False)
    self._isSuctionEnabled = False

  def reset(self) -> None:
    self._resetGripper()
    self._resetSuction()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsEnabled", self._isGripperEnabled)
    SmartDashboard.putNumber("Robot/Hand/Gripper/Speed", self._gripperMotor.get())
    SmartDashboard.putNumber("Robot/Hand/Gripper/Current", self._gripperMotor.getOutputCurrent()) 
    SmartDashboard.putBoolean("Robot/Hand/Suction/IsEnabled", self._isSuctionEnabled)
    SmartDashboard.putNumber("Robot/Hand/Suction/Speed", self._suctionMotor.get())
    SmartDashboard.putNumber("Robot/Hand/Suction/Current", self._suctionMotor.getOutputCurrent())
 
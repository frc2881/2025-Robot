from typing import Callable
import math
from wpilib import SmartDashboard, Solenoid, Timer
from wpilib import PneumaticsModuleType
from wpimath import units
from commands2 import Subsystem, Command, cmd
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class HandSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Hand

    self._isGripperEnabled: bool = False
    self._isGripperHolding: bool = False
    self._isGripperDisabling: bool = False
    self._isSuctionEnabled: bool = False
    self._isSuctionDisabling: bool = False

    self._gripperIntakeStartTime: units.seconds = 0
    self._gripperEjectStartTime: units.seconds = 0

    self._gripperMotor = SparkMax(self._constants.kGripperMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kGripperMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kGripperMotorCurrentLimit)
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
      .secondaryCurrentLimit(self._constants.kSuctionMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._suctionMotor.configure(
        self._sparkConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._suctionSolenoid = Solenoid(PneumaticsModuleType.REVPH, self._constants.kSuctionSolenoidPortId)

  def periodic(self) -> None:
    if self._isGripperEnabled:
      if not self._isGripperHolding:
        self._gripperMotor.set(self._constants.kGripperIntakeSpeed)
        if math.fabs(self._gripperIntakeStartTime - Timer.getFPGATimestamp()) >= 0.2 and self._gripperMotor.getOutputCurrent() > self._constants.kGripperMaxCurrent:
          self._isGripperHolding = True
      else:
        self._gripperMotor.set(self._constants.kGripperHoldSpeed)
    else:
      if self._isGripperDisabling:
        self._gripperMotor.set(self._constants.kGripperEjectSpeed)
        if math.fabs(self._gripperEjectStartTime - Timer.getFPGATimestamp()) >= 0.5:
          self._isGripperDisabling = False
      else:
        self.resetGripper()

    # if self._isSuctionEnabled:
    #   self._suctionMotor.set(self._constants.kSuctionMotorSpeed)
    # else:
    #   if self._isSuctionDisabling:
    #     self._suctionMotor.set(0)
    #     # self._suctionSolenoid.set(False) #TODO: Fix the suction solenoid and test whether it needs to on or off to release suction
    #     self._isSuctionDisabling = False
    #   else:
    #     self.resetSuction()

    self._updateTelemetry()
  
  def enableGripper(self) -> None:
    self._isGripperEnabled = True
    self._gripperIntakeStartTime = Timer.getFPGATimestamp()

  def disableGripper(self) -> None:
    self._isGripperEnabled = False
    self._isGripperDisabling = True
    self._gripperEjectStartTime = Timer.getFPGATimestamp()

  def toggleGripperCommand(self) -> Command:
    return cmd.either(
      cmd.runOnce(
        lambda: self.disableGripper()
      ), cmd.runOnce(
        lambda: self.enableGripper()
      ), lambda: self._isGripperEnabled
    ).withName("HandSubsystem:ToggleGripper")

  def enableSuction(self) -> None:
    self._isSuctionEnabled = True

  def disableSuction(self) -> None:
    self._isSuctionEnabled = False
    self._isSuctionDisabling = True

  # def toggleSuctionCommand(self) -> Command:
  #   return cmd.either(
  #     self.disableSuction, self.enableSuction, lambda: self._isSuctionEnabled
  #   ).withName("HandSubsystem:ToggleSuction")
  
  def resetGripper(self) -> None:
    self._isGripperEnabled = False
    self._isGripperHolding = False
    self._isGripperDisabling = False
    self._gripperMotor.stopMotor()

  def resetSuction(self) -> None:
    self._isSuctionEnabled = False
    self._isSuctionDisabling = False
    # self._suctionSolenoid.set(True)
    self._suctionMotor.stopMotor()
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Hand/Gripper/Speed", self._gripperMotor.get())
    SmartDashboard.putNumber("Robot/Hand/Gripper/Current", self._gripperMotor.getOutputCurrent())
    SmartDashboard.putBoolean("Robot/Hand/Gripper/IsGripperHolding", self._isGripperHolding)
    SmartDashboard.putNumber("Robot/Hand/Suction/Speed", self._suctionMotor.get())
 
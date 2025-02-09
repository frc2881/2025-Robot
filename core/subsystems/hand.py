from typing import Callable
from wpilib import SmartDashboard, Solenoid
from wpilib import PneumaticsModuleType
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
import core.constants as constants

class HandSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Hand

    self._rollerMotor = SparkMax(self._constants.kRollerMotorCANId, SparkBase.MotorType.kBrushed)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kHandMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kHandMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._rollerMotor.configure(
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

  def runRollerCommand(self, speed: units.percent) -> Command:
    return self.run(
      lambda: self._rollerMotor.set(speed)
    ).finallyDo(
      lambda end: self.resetRoller()
    ).withName("HandSubsystem:RunRoller")
  
  def runSuctionCommand(self) -> Command:
    return self.run(
      lambda: self._suctionMotor.set(self._constants.kSuctionMotorSpeed)
    ).finallyDo(
      lambda end: self.resetSuction()
    ).until(
      lambda: self._suctionMotor.getOutputCurrent() > self._constants.kSuctionMaxCurrent
    ).withName("HandSubsystem:RunSuction")
  
  def releaseSuctionCommand(self) -> Command:
    return self.run(
      lambda: self._suctionSolenoid.set(False)
    ).finallyDo(
      lambda end: self.resetSolenoid()
    ).withName("HandSubsystem:ReleaseSuction")
    
  def periodic(self) -> None:
    self._updateTelemetry()
  
  def resetRoller(self) -> None:
    self._rollerMotor.stopMotor()

  def resetSuction(self) -> None:
    self._suctionMotor.stopMotor()

  def resetSolenoid(self) -> None:
    self._suctionSolenoid.set(True)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Hand/Roller/Speed", self._rollerMotor.get())
    SmartDashboard.putNumber("Robot/Hand/Suction/Speed", self._suctionMotor.get())
 
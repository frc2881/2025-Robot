import math
from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase, SparkBaseConfig, ClosedLoopConfig
from lib import logger, utils
import core.constants as constants

class ArmSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Arm

    self._hasInitialZeroReset: bool = False
    self._isAlignedToPosition: bool = False

    self._armMotor = SparkMax(self._constants.kArmMotorCANId, SparkBase.MotorType.kBrushless)
    self._motorConfig = SparkBaseConfig()
    (self._motorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kMotorCurrentLimit))
    (self._motorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pid(*self._constants.kMotorPID)
      .outputRange(-1.0, 1.0)
      .maxMotion
        # .maxVelocity(motorMotionMaxVelocity) #TODO: Check if we need motorMotionMaxVelocity and the value
        # .maxAcceleration(motorMotionMaxAcceleration) #TODO: Check if we need motorMotionMaxAcceleration and the value
        .allowedClosedLoopError(self._constants.kAllowedClosedLoopError))
    (self._motorConfig.softLimit
      .forwardSoftLimitEnabled(True)
      .forwardSoftLimit(self._constants.kMotorSoftLimitForward)
      .reverseSoftLimitEnabled(True)
      .reverseSoftLimit(self._constants.kMotorSoftLimitReverse))
     
    utils.setSparkConfig(
      self._armMotor.configure(
        self._motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._closedLoopController = self._armMotor.getClosedLoopController()

    self._encoder = self._armMotor.getEncoder()
    self._encoder.setPosition(0)

  def runCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._setSpeed(getInput() * self._constants.kInputLimit)
    ).finallyDo(
      lambda end: self.reset()
    ).withName("ArmSubsystem:Run")
  
  def _setSpeed(self, speed: units.percent) -> None:
    self._armMotor.set(speed)

  def alignToPositiontCommand(self, position: units.meters) -> Command:
    return self.run(
      lambda: [
        self._setPosition(position),
        self._setIsAlignedToPosition(position)
      ]
    ).beforeStarting(
      lambda: self.clearPositionAlignment()
    ).withName("ArmSubsystem:AlignToPosition")
  
  def _setPosition(self, position: float) -> None:
    self._closedLoopController.setReference(position, SparkBase.ControlType.kMAXMotionPositionControl)

  def _setIsAlignedToPosition(self, position: float) -> None:
    self._isAlignedToPosition = math.fabs(self._getPosition() - position) <= self._constants.kHeightAlignmentPositionTolerance

  def _getPosition(self) -> float:
    return self._encoder.getPosition()
    
  def periodic(self) -> None:
    self._updateTelemetry()
  
  def clearPositionAlignment(self) -> None:
    self._isAlignedToPosition = False
  
  def reset(self) -> None:
    self._armMotor.stopMotor()

  def _startZeroReset(self) -> None:
    utils.setSparkSoftLimitsEnabled(self._armMotor, False)
    self._armMotor.set(-self._constants.kMotorResetSpeed)

  def _endZeroReset(self) -> None:
    self._armMotor.stopMotor()
    self._encoder.setPosition(0)
    utils.setSparkSoftLimitsEnabled(self._armMotor, True)

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._startZeroReset()
      ],
      lambda: [
        self._endZeroReset(),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("ArmSubsystem:ResetToZero")

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Arm/Position", self._getPosition())
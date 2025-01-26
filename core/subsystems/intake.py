import math
from typing import Callable
from wpilib import SmartDashboard
from wpimath import units
from commands2 import Subsystem, Command
from rev import SparkMax, SparkMaxConfig, SparkBase
from lib import logger, utils
from lib.components.position_control_module import PositionControlModule
import core.constants as constants

class IntakeSubsystem(Subsystem):
  def __init__(self):
    super().__init__()
    self._constants = constants.Subsystems.Intake

    self._hasInitialZeroReset: bool = False
    self._isAlignedToTarget: bool = False

    self._leadscrewModuleLeft = PositionControlModule(self._constants.kLeadScrewModuleConfigLeft)
    self._leadscrewModuleRight = PositionControlModule(self._constants.kLeadScrewModuleConfigRight)

    self._rollerMotor = SparkMax(self._constants.kRollerMotorCANId, SparkBase.MotorType.kBrushless)
    self._sparkConfig = SparkMaxConfig()
    (self._sparkConfig
      .smartCurrentLimit(self._constants.kRollerMotorCurrentLimit)
      .inverted(True))
    utils.setSparkConfig(
      self._rollerMotor.configure(
        self._sparkConfig, 
        SparkBase.ResetMode.kResetSafeParameters, 
        SparkBase.PersistMode.kPersistParameters
      )
    )

  def periodic(self) -> None:
    self._updateTelemetry()
  
  def moveIntakeCommand(self, getInput: Callable[[], units.percent]) -> Command:
    return self.run(
      lambda: self._setLeadscrewSpeed(getInput() * self._constants.kInputLimit)
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).finallyDo(
      lambda end: self.reset()
    ).withName("IntakeSubsystem:MoveIntake")
  
  def runRollersCommand(self, speed: float):
    return self.run(
      self._rollerMotor.set(speed)
    ).finallyDo(
      self._rollerMotor.stopMotor()
    ).withName("IntakeSubsystem:RunRollers")

  def alignToPositionCommand(self, position: float) -> Command:
    return self.run(
      lambda: [
        self._setPosition(position),
        self._setIsAlignedToTarget(position)
      ]
    ).beforeStarting(
      lambda: self.clearTargetAlignment()
    ).withName("IntakeSubsystem:AlignToPosition")

  def _setLeadscrewSpeed(self, speed: units.percent) -> None:
    self._leadscrewModuleLeft.setSpeed(speed)

  def _setPosition(self, position: float) -> None:
    self._leadscrewModuleLeft.setPosition(position)

  def _getPosition(self) -> float:
    return self._leadscrewModuleLeft.getPosition()

  def _setIsAlignedToTarget(self, position: float) -> None:
    self._isAlignedToTarget = math.fabs(self._getPosition() - position) <= self._constants.kTargetAlignmentPositionTolerance

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def resetToZeroCommand(self) -> Command:
    return self.startEnd(
      lambda: [
        self._leadscrewModuleLeft.startZeroReset(),
        self._leadscrewModuleRight.startZeroReset()
      ],
      lambda: [
        self._leadscrewModuleLeft.endZeroReset(),
        self._leadscrewModuleRight.endZeroReset(),
        setattr(self, "_hasInitialZeroReset", True)
      ]
    ).withName("IntakeSubsystem:ResetToZero")

  def hasInitialZeroReset(self) -> bool:
    return self._hasInitialZeroReset

  def reset(self) -> None:
    self._leadscrewModuleLeft.reset()
    self._leadscrewModuleRight.reset()
    self.clearTargetAlignment()
    self._rollerMotor.stopMotor()

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber("Robot/Intake/Position", self._getPosition())
    SmartDashboard.putBoolean("Robot/Intake/IsAlignedToTarget", self._isAlignedToTarget)
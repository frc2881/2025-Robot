from typing import Callable
from commands2 import Subsystem, Command
from wpilib import SmartDashboard, SendableChooser
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState, SwerveDrive4Kinematics
from pathplannerlib.util import DriveFeedforwards
from lib import logger, utils
from lib.classes import MotorIdleMode, SpeedMode, DriveOrientation, OptionState, LockState, TargetAlignmentMode
from lib.components.swerve_module import SwerveModule
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants

class DriveSubsystem(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._swerveModules = tuple(SwerveModule(c) for c in self._constants.kSwerveModuleConfigs)

    self._isDriftCorrectionActive: bool = False
    self._driftCorrectionController = PIDController(*self._constants.kDriftCorrectionConstants.rotationPID)
    self._driftCorrectionController.setTolerance(*self._constants.kDriftCorrectionConstants.rotationTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._isAlignedToTarget: bool = False
    self._targetAlignmentRotationController = PIDController(*self._constants.kTargetAlignmentConstants.rotationPID)
    self._targetAlignmentRotationController.setTolerance(*self._constants.kTargetAlignmentConstants.rotationTolerance)
    self._targetAlignmentRotationController.enableContinuousInput(-180.0, 180.0)
    self._targetAlignmentTranslationXController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationXController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationXController.setSetpoint(0)
    self._targetAlignmentTranslationYController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationYController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationYController.setSetpoint(0)
    self._targetPose: Pose3d = None

    self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedModeChooser = SendableChooser()
    speedModeChooser.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedModeChooser.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedModeChooser.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", speedModeChooser)

    self._orientation: DriveOrientation = DriveOrientation.Field
    orientationChooser = SendableChooser()
    orientationChooser.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientationChooser.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientationChooser.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    SmartDashboard.putData("Robot/Drive/Orientation", orientationChooser)

    self._driftCorrection: OptionState = OptionState.Enabled
    driftCorrectionChooser = SendableChooser()
    driftCorrectionChooser.setDefaultOption(OptionState.Enabled.name, OptionState.Enabled)
    driftCorrectionChooser.addOption(OptionState.Disabled.name, OptionState.Disabled)
    driftCorrectionChooser.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrectionChooser)

    idleModeChooser = SendableChooser()
    idleModeChooser.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleModeChooser.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleModeChooser.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleModeChooser)

    self._lockState: LockState = LockState.Unlocked

    SmartDashboard.putNumber("Robot/Drive/Chassis/Length", self._constants.kWheelBase)
    SmartDashboard.putNumber("Robot/Drive/Chassis/Width", self._constants.kTrackWidth)
    SmartDashboard.putNumber("Robot/Drive/Speed/Max", self._constants.kTranslationSpeedMax)

  def periodic(self) -> None:
    self._updateTelemetry()

  def driveCommand(
      self, 
      getInputX: Callable[[], units.percent], 
      getInputY: Callable[[], units.percent], 
      getInputRotation: Callable[[], units.percent]
    ) -> Command:
    return self.run(
      lambda: self._drive(getInputX(), getInputY(), getInputRotation())
    ).onlyIf(
      lambda: self._lockState != LockState.Locked
    ).withName("DriveSubsystem:Drive")

  def drive(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: DriveFeedforwards = None) -> None:
    self._setSwerveModuleStates(
      self._constants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(chassisSpeeds, 0.02)
      )
    )

  def _drive(self, inputX: units.percent, inputY: units.percent, inputRotation: units.percent) -> None:
    if self._driftCorrection == OptionState.Enabled:
      isTranslating: bool = inputX != 0 or inputY != 0
      isRotating: bool = inputRotation != 0
      if isTranslating and not isRotating and not self._isDriftCorrectionActive:
        self._isDriftCorrectionActive = True
        self._driftCorrectionController.reset()
        self._driftCorrectionController.setSetpoint(self._getGyroHeading())
      elif isRotating or not isTranslating:
        self._isDriftCorrectionActive = False
      if self._isDriftCorrectionActive:
        inputRotation = self._driftCorrectionController.calculate(self._getGyroHeading())
        if self._driftCorrectionController.atSetpoint():
          inputRotation = 0

    if self._speedMode == SpeedMode.Demo:
      inputX = self._inputXFilter.calculate(inputX * self._constants.kInputLimitDemo)
      inputY = self._inputYFilter.calculate(inputY * self._constants.kInputLimitDemo)
      inputRotation = self._inputRotationFilter.calculate(inputRotation * self._constants.kInputLimitDemo)

    speedX: units.meters_per_second = inputX * self._constants.kTranslationSpeedMax
    speedY: units.meters_per_second = inputY * self._constants.kTranslationSpeedMax
    speedRotation: units.radians_per_second = inputRotation * self._constants.kRotationSpeedMax # type: ignore
    
    if self._orientation == DriveOrientation.Field:
      self.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRotation, Rotation2d.fromDegrees(self._getGyroHeading())))
    else:
      self.drive(ChassisSpeeds(speedX, speedY, speedRotation))      

  def _setSwerveModuleStates(self, swerveModuleStates: tuple[SwerveModuleState, ...]) -> None:
    SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, self._constants.kTranslationSpeedMax)
    for i, m in enumerate(self._swerveModules):
      m.setTargetState(swerveModuleStates[i])

  def _getSwerveModuleStates(self) -> tuple[SwerveModuleState, ...]:
    return tuple(m.getState() for m in self._swerveModules)

  def getModulePositions(self) -> tuple[SwerveModulePosition, ...]:
    return tuple(m.getPosition() for m in self._swerveModules)

  def getChassisSpeeds(self) -> ChassisSpeeds:
    return self._constants.kDriveKinematics.toChassisSpeeds(self._getSwerveModuleStates())

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    for m in self._swerveModules: m.setIdleMode(idleMode)
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)

  def lockCommand(self) -> Command:
    return self.startEnd(
      lambda: self._setLockState(LockState.Locked),
      lambda: self._setLockState(LockState.Unlocked)
    ).withName("DriveSubsystem:Lock")
  
  def _setLockState(self, lockState: LockState) -> None:
    self._lockState = lockState
    if lockState == LockState.Locked:
      for i, m in enumerate(self._swerveModules): 
        m.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45 if i in { 0, 3 } else -45)))

  def alignToTargetCommand(
      self, 
      getRobotPose: Callable[[], Pose2d], 
      getTargetPose: Callable[[TargetAlignmentLocation], Pose3d], 
      targetAlignmentMode: TargetAlignmentMode, 
      targetAlignmentLocation: TargetAlignmentLocation,
      targetType: TargetType
    ) -> Command:
    return self.run(
      lambda: self._runTargetAlignment(getRobotPose(), targetAlignmentMode)
    ).beforeStarting(
      lambda: self._initTargetAlignment(getRobotPose(), getTargetPose(targetAlignmentLocation, targetType), targetAlignmentMode)
    ).onlyIf(
      lambda: self._lockState != LockState.Locked
    ).until(
      lambda: self._isAlignedToTarget
    ).withName("DriveSubsystem:AlignToTarget")
  
  def _initTargetAlignment(
      self, 
      robotPose: Pose2d, 
      targetPose: Pose3d, 
      targetAlignmentMode: TargetAlignmentMode
    ) -> None:
    self.clearTargetAlignment()
    self._targetPose = targetPose
    self._targetAlignmentRotationController.reset()
    if targetAlignmentMode == TargetAlignmentMode.Heading:
      self._targetAlignmentRotationController.setSetpoint(utils.wrapAngle(utils.getTargetHeading(robotPose, targetPose) + self._constants.kTargetAlignmentConstants.rotationHeadingModeOffset))
    else:
      self._targetAlignmentRotationController.setSetpoint(targetPose.toPose2d().rotation().degrees() + self._constants.kTargetAlignmentConstants.rotationTranslationModeOffset)
    self._targetAlignmentTranslationXController.reset()
    self._targetAlignmentTranslationYController.reset()
    
  def _runTargetAlignment(self, robotPose: Pose2d, targetAlignmentMode: TargetAlignmentMode) -> None:
    targetTranslation = self._targetPose.__sub__(Pose3d(robotPose))

    speedRotation = 0
    speedTranslationX = 0
    speedTranslationY = 0

    if not self._targetAlignmentRotationController.atSetpoint():
      speedRotation = self._targetAlignmentRotationController.calculate(robotPose.rotation().degrees())

    if targetAlignmentMode == TargetAlignmentMode.Translation and not self._targetAlignmentTranslationXController.atSetpoint():
      speedTranslationX = self._targetAlignmentTranslationXController.calculate(targetTranslation.X())

    if targetAlignmentMode == TargetAlignmentMode.Translation and not self._targetAlignmentTranslationYController.atSetpoint():
      speedTranslationY = self._targetAlignmentTranslationYController.calculate(targetTranslation.Y())

    self._setSwerveModuleStates(
      self._constants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds(
          -utils.clampValue(speedTranslationX, -self._constants.kTargetAlignmentConstants.translationSpeedMax, self._constants.kTargetAlignmentConstants.translationSpeedMax), 
          -utils.clampValue(speedTranslationY, -self._constants.kTargetAlignmentConstants.translationSpeedMax, self._constants.kTargetAlignmentConstants.translationSpeedMax),
          utils.clampValue(speedRotation, -self._constants.kTargetAlignmentConstants.rotationSpeedMax, self._constants.kTargetAlignmentConstants.rotationSpeedMax)
        )
      )
    )

    if speedRotation == 0 and speedTranslationX == 0 and speedTranslationY == 0:
      self._isAlignedToTarget = True

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False
    self._targetPose = None

  def reset(self) -> None:
    self.drive(ChassisSpeeds())
    self.clearTargetAlignment()
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putString("Robot/Drive/LockState", self._lockState.name)
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self._isAlignedToTarget)
  
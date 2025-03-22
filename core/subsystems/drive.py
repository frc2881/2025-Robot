from typing import Callable
from ntcore import NetworkTableInstance
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
import core.constants as constants

class Drive(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._swerveModules = tuple(SwerveModule(c) for c in self._constants.kSwerveModuleConfigs)
    self._swerveModuleStatesPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SmartDashboard/Robot/Drive/Modules/States", SwerveModuleState).publish()

    self._isDriftCorrectionActive: bool = False
    self._driftCorrectionController = PIDController(*self._constants.kDriftCorrectionConstants.rotationPID)
    self._driftCorrectionController.setTolerance(*self._constants.kDriftCorrectionConstants.rotationTolerance)
    self._driftCorrectionController.enableContinuousInput(-180.0, 180.0)

    self._targetPose: Pose3d = None
    self._isAligningToTarget: bool = False
    self._isAlignedToTarget: bool = False
    self._targetAlignmentTranslationXController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationXController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentTranslationYController = PIDController(*self._constants.kTargetAlignmentConstants.translationPID)
    self._targetAlignmentTranslationYController.setTolerance(*self._constants.kTargetAlignmentConstants.translationTolerance)
    self._targetAlignmentRotationController = PIDController(*self._constants.kTargetAlignmentConstants.rotationPID)
    self._targetAlignmentRotationController.setTolerance(*self._constants.kTargetAlignmentConstants.rotationTolerance)
    self._targetAlignmentRotationController.enableContinuousInput(-180.0, 180.0)
    
    self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedMode = SendableChooser()
    speedMode.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedMode.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedMode.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", speedMode)

    self._orientation: DriveOrientation = DriveOrientation.Field
    orientation = SendableChooser()
    orientation.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientation.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientation.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    SmartDashboard.putData("Robot/Drive/Orientation", orientation)

    self._driftCorrection: OptionState = OptionState.Enabled
    driftCorrection = SendableChooser()
    driftCorrection.setDefaultOption(OptionState.Enabled.name, OptionState.Enabled)
    driftCorrection.addOption(OptionState.Disabled.name, OptionState.Disabled)
    driftCorrection.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrection)

    idleMode = SendableChooser()
    idleMode.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleMode.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    idleMode.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleMode)

    self._lockState: LockState = LockState.Unlocked

    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotLength", self._constants.kRobotLength)
    SmartDashboard.putNumber("Robot/Drive/Chassis/RobotWidth", self._constants.kRobotWidth)

  def periodic(self) -> None:
    self._updateTelemetry()

  def default(
      self, 
      getInputX: Callable[[], units.percent], 
      getInputY: Callable[[], units.percent], 
      getInputRotation: Callable[[], units.percent]
    ) -> Command:
    return self.run(
      lambda: self._drive(getInputX(), getInputY(), getInputRotation())
    ).onlyIf(
      lambda: self._lockState != LockState.Locked
    ).withName("Drive:Run")

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
    speedRotation: units.radians_per_second = inputRotation * self._constants.kRotationSpeedMax
    
    if self._orientation == DriveOrientation.Field:
      self.drive(ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedRotation, Rotation2d.fromDegrees(self._getGyroHeading())))
    else:
      self.drive(ChassisSpeeds(speedX, speedY, speedRotation))      

  def drive(self, chassisSpeeds: ChassisSpeeds, driveFeedforwards: DriveFeedforwards = None) -> None:
    self._setSwerveModuleStates(chassisSpeeds)
    if chassisSpeeds.vx != 0 or chassisSpeeds.vy != 0:
      self._resetTargetAlignment()

  def _setSwerveModuleStates(self, chassisSpeeds: ChassisSpeeds) -> None: 
    swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(
      self._constants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
          self._constants.kDriveKinematics.toChassisSpeeds(
            SwerveDrive4Kinematics.desaturateWheelSpeeds(
              self._constants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds), 
              self._constants.kTranslationSpeedMax
            )
          ), 0.02
        )
      ), self._constants.kTranslationSpeedMax
    )
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

  def lock(self) -> Command:
    return self.startEnd(
      lambda: self._setLockState(LockState.Locked),
      lambda: self._setLockState(LockState.Unlocked)
    ).withName("Drive:Lock")
  
  def _setLockState(self, lockState: LockState) -> None:
    self._lockState = lockState
    if lockState == LockState.Locked:
      for i, m in enumerate(self._swerveModules): 
        m.setTargetState(SwerveModuleState(0, Rotation2d.fromDegrees(45 if i in { 0, 3 } else -45)))

  def alignToTarget(
      self, 
      getRobotPose: Callable[[], Pose2d], 
      getTargetPose: Callable[[], Pose3d], 
      targetAlignmentMode: TargetAlignmentMode
    ) -> Command:
    return self.startRun(
      lambda: self._initTargetAlignment(getRobotPose(), getTargetPose(), targetAlignmentMode),
      lambda: self._runTargetAlignment(getRobotPose(), targetAlignmentMode)
    ).onlyIf(
      lambda: self._lockState != LockState.Locked
    ).withName("Drive:AlignToTarget")
  
  def _initTargetAlignment(
      self, 
      robotPose: Pose2d, 
      targetPose: Pose3d, 
      targetAlignmentMode: TargetAlignmentMode
    ) -> None:
    self._resetTargetAlignment()
    self._isAligningToTarget = True
    self._targetPose = targetPose
    self._targetAlignmentTranslationXController.reset()
    self._targetAlignmentTranslationXController.setSetpoint(0)
    self._targetAlignmentTranslationYController.reset()
    self._targetAlignmentTranslationYController.setSetpoint(0)
    self._targetAlignmentRotationController.reset()
    self._targetAlignmentRotationController.setSetpoint(
      utils.wrapAngle(utils.getTargetHeading(robotPose, targetPose) + self._constants.kTargetAlignmentConstants.rotationHeadingModeOffset)
      if targetAlignmentMode == TargetAlignmentMode.Heading else
      targetPose.toPose2d().rotation().degrees() + self._constants.kTargetAlignmentConstants.rotationTranslationModeOffset
    )

  def _runTargetAlignment(self, robotPose: Pose2d, targetAlignmentMode: TargetAlignmentMode) -> None:
    speedTranslationX = 0
    speedTranslationY = 0
    speedRotation = 0
    if targetAlignmentMode == TargetAlignmentMode.Translation:
      targetTranslation = self._targetPose.toPose2d() - robotPose
      if not self._targetAlignmentTranslationXController.atSetpoint():
        speedTranslationX = -utils.clampValue(
          self._targetAlignmentTranslationXController.calculate(targetTranslation.X()), 
          -self._constants.kTargetAlignmentConstants.translationSpeedMax, 
          self._constants.kTargetAlignmentConstants.translationSpeedMax
        )
      if not self._targetAlignmentTranslationYController.atSetpoint():
        speedTranslationY = -utils.clampValue(
          self._targetAlignmentTranslationYController.calculate(targetTranslation.Y()), 
          -self._constants.kTargetAlignmentConstants.translationSpeedMax, 
          self._constants.kTargetAlignmentConstants.translationSpeedMax
        )
    if not self._targetAlignmentRotationController.atSetpoint():
      speedRotation = utils.clampValue(
        self._targetAlignmentRotationController.calculate(robotPose.rotation().degrees()), 
        -self._constants.kTargetAlignmentConstants.rotationSpeedMax, 
        self._constants.kTargetAlignmentConstants.rotationSpeedMax
      )
    self._setSwerveModuleStates(ChassisSpeeds(speedTranslationX, speedTranslationY, speedRotation))
    if speedRotation == 0 and speedTranslationX == 0 and speedTranslationY == 0:
      self._isAlignedToTarget = True
      self._isAligningToTarget = False

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def isAligningToTarget(self) -> bool:
    return self._isAligningToTarget
  
  def _resetTargetAlignment(self) -> None:
    self._targetPose = None
    self._isAlignedToTarget = False
    self._isAligningToTarget = False

  def reset(self) -> None:
    self.drive(ChassisSpeeds())
    self._resetTargetAlignment()
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self._isAlignedToTarget)
    SmartDashboard.putBoolean("Robot/Drive/IsAligningToTarget", self._isAligningToTarget)
    SmartDashboard.putString("Robot/Drive/LockState", self._lockState.name)
    self._swerveModuleStatesPublisher.set(self._getSwerveModuleStates())

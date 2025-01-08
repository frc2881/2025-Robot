from typing import Callable
import math
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib.sensors.pose_sensor import PoseSensor
from lib.classes import TargetInfo
from lib import logger, utils
import constants

class LocalizationSubsystem(Subsystem):
  def __init__(
      self,
      poseSensors: tuple[PoseSensor, ...],
      getGyroRotation: Callable[[], Rotation2d],
      getModulePositions: Callable[[], tuple[SwerveModulePosition, ...]]
    ) -> None:
    super().__init__()
    self._poseSensors = poseSensors
    self._getGyroRotation = getGyroRotation
    self._getModulePositions = getModulePositions

    self._poseEstimator = SwerveDrive4PoseEstimator(
      constants.Subsystems.Drive.kDriveKinematics,
      self._getGyroRotation(),
      self._getModulePositions(),
      Pose2d()
    )

    self._pose = Pose2d()
    self._targetPose = Pose3d()
    self._targetInfo = TargetInfo(0, 0, 0)
    self._currentAlliance = None

    SmartDashboard.putNumber("Robot/Game/Field/Length", constants.Game.Field.kLength)
    SmartDashboard.putNumber("Robot/Game/Field/Width", constants.Game.Field.kWidth)

    utils.addRobotPeriodic(lambda: [ 
      self._updatePose(),
      self._updateTargetPose(),
      self._updateTargetInfo(),
      self._updateTelemetry()
    ], 0.033)

  def periodic(self) -> None:
    pass

  def _updatePose(self) -> None:
    self._poseEstimator.update(self._getGyroRotation(), self._getModulePositions())
    for poseSensor in self._poseSensors:
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        pose = estimatedRobotPose.estimatedPose.toPose2d()
        if utils.isPoseInBounds(pose, constants.Game.Field.kBounds):
          if estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR:
            self._poseEstimator.addVisionMeasurement(
              pose,
              estimatedRobotPose.timestampSeconds,
              constants.Subsystems.Localization.kMultiTagStandardDeviations
            )
          else:
            for target in estimatedRobotPose.targetsUsed:
              if utils.isValueInRange(target.getPoseAmbiguity(), 0, constants.Subsystems.Localization.kMaxPoseAmbiguity):
                self._poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds, constants.Subsystems.Localization.kSingleTagStandardDeviations)
                break
    self._pose = self._poseEstimator.getEstimatedPosition()

  def getPose(self) -> Pose2d:
    return self._pose

  def resetPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)

  def hasVisionTargets(self) -> bool:
    for poseSensor in self._poseSensors:
      if poseSensor.hasTarget():
        return True
    return False

  def _updateTargetPose(self) -> None:
    if utils.getAlliance() != self._currentAlliance:
      self._currentAlliance = utils.getAlliance()
      self._targetPose = utils.getValueForAlliance(
        constants.Game.Field.Targets.kBlueTarget, 
        constants.Game.Field.Targets.kRedTarget
      ).transformBy(
        constants.Game.Field.Targets.kTargetTransform
      )

  def _updateTargetInfo(self) -> None:
    self._targetInfo.distance = self._pose.translation().distance(self._targetPose.toPose2d().translation())
    translation = self._targetPose.toPose2d().relativeTo(self._pose).translation()
    rotation = Rotation2d(translation.X(), translation.Y()).rotateBy(self._pose.rotation())
    self._targetInfo.heading = utils.wrapAngle(rotation.degrees())
    self._targetInfo.pitch = math.degrees(math.atan2((self._targetPose - Pose3d(self._pose)).Z(), self._targetInfo.distance))

  def getTargetDistance(self) -> units.meters:
    return self._targetInfo.distance

  def getTargetHeading(self) -> units.degrees:
    return self._targetInfo.heading
  
  def getTargetPitch(self) -> units.degrees:
    return self._targetInfo.pitch
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumberArray("Robot/Localization/Pose", [self._pose.X(), self._pose.Y(), self._pose.rotation().degrees()])
    SmartDashboard.putNumber("Robot/Localization/Target/Distance", self._targetInfo.distance)
    SmartDashboard.putNumber("Robot/Localization/Target/Heading", self._targetInfo.heading)
    SmartDashboard.putNumber("Robot/Localization/Target/Pitch", self._targetInfo.pitch)

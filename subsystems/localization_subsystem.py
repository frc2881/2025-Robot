from typing import Callable
import math
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d, Pose3d, Transform3d
from wpimath.kinematics import SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib.sensors.pose_sensor import PoseSensor
from lib import logger, utils
from classes import Target, TargetAlignmentLocation, TargetAlignmentInfo
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

    self._robotPose = Pose2d()
    self._targets: dict[int, Target] = {}
    self._targetPoses: list[Pose2d] = []
    self._alliance = None

    SmartDashboard.putNumber("Robot/Game/Field/Length", constants.Game.Field.kLength)
    SmartDashboard.putNumber("Robot/Game/Field/Width", constants.Game.Field.kWidth)

  def periodic(self) -> None:
    self._updateRobotPose()
    self._updateTargets()
    self._updateTelemetry()

  def _updateRobotPose(self) -> None:
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
    self._robotPose = self._poseEstimator.getEstimatedPosition()

  def getRobotPose(self) -> Pose2d:
    return self._robotPose

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)

  def hasTarget(self) -> bool:
    for poseSensor in self._poseSensors:
      if poseSensor.hasTarget():
        return True
    return False

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.kTargets[self._alliance]
      self._targetPoses = [t.pose.toPose2d() for t in self._targets.values()]

  def getTargetPose(self, targetAlignmentLocation: TargetAlignmentLocation) -> Pose3d:
    target = self._targets.get(utils.getTargetHash(self._robotPose.nearest(self._targetPoses)))
    return target.pose.transformBy(constants.Game.Field.Targets.kTargetAlignmentTransforms[target.type][targetAlignmentLocation])

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumberArray("Robot/Localization/Pose", [self._robotPose.X(), self._robotPose.Y(), self._robotPose.rotation().degrees()])

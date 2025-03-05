from typing import Callable
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Pose2d, Pose3d
from wpimath.kinematics import SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib.sensors.pose import PoseSensor
from lib import logger, utils
from core.classes import Target, TargetType, TargetAlignmentLocation
import core.constants as constants

class Localization():
  def __init__(
      self,
      getGyroRotation: Callable[[], Rotation2d],
      getModulePositions: Callable[[], tuple[SwerveModulePosition, ...]],
      poseSensors: tuple[PoseSensor, ...]
    ) -> None:
    super().__init__()
    self._getGyroRotation = getGyroRotation
    self._getModulePositions = getModulePositions
    self._poseSensors = poseSensors

    self._poseEstimator = SwerveDrive4PoseEstimator(
      constants.Subsystems.Drive.kDriveKinematics,
      self._getGyroRotation(),
      self._getModulePositions(),
      Pose2d(),
      constants.Services.Localization.kStateStandardDeviations,
      constants.Services.Localization.kVisionStandardDeviations
    )

    self._alliance = None
    self._robotPose = Pose2d()
    self._targets: dict[int, Target] = {}
    self._targetPoses: list[Pose2d] = []
    
    self._robotPosePublisher = NetworkTableInstance.getDefault().getStructTopic("/SmartDashboard/Robot/Localization/Pose", Pose2d).publish()
    SmartDashboard.putNumber("Game/Field/Length", constants.Game.Field.kLength)
    SmartDashboard.putNumber("Game/Field/Width", constants.Game.Field.kWidth)

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateRobotPose()
    self._updateTargets()
    self._updateTelemetry()

  def _updateRobotPose(self) -> None:
    self._poseEstimator.update(self._getGyroRotation(), self._getModulePositions())
    for poseSensor in self._poseSensors:
      # TODO: add check if robot is actively aligning to target and if true exclude rear cameras temporarily from this processing loop (only use forward cameras during robot target alignment)
      estimatedRobotPose = poseSensor.getEstimatedRobotPose()
      if estimatedRobotPose is not None:
        pose = estimatedRobotPose.estimatedPose.toPose2d()
        if utils.isPoseInBounds(pose, constants.Game.Field.kBounds):
          if estimatedRobotPose.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR:
            self._poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds)
          else:
            for target in estimatedRobotPose.targetsUsed:
              if utils.isValueInRange(target.getPoseAmbiguity(), 0, constants.Services.Localization.kVisionMaxPoseAmbiguity):
                self._poseEstimator.addVisionMeasurement(pose, estimatedRobotPose.timestampSeconds)
                break
    self._robotPose = self._poseEstimator.getEstimatedPosition()

  def getRobotPose(self) -> Pose2d:
    return self._robotPose

  def resetRobotPose(self, pose: Pose2d) -> None:
    self._poseEstimator.resetPose(pose)

  def _updateTargets(self) -> None:
    if utils.getAlliance() != self._alliance:
      self._alliance = utils.getAlliance()
      self._targets = constants.Game.Field.Targets.kTargets[self._alliance]
      self._targetPoses = [t.pose.toPose2d() for t in self._targets.values()]

  def getTargetPose(self, targetAlignmentLocation: TargetAlignmentLocation, isElevatorReefCoralL4Ready: bool) -> Pose3d:
    target = self._targets.get(utils.getTargetHash(self._robotPose.nearest(self._targetPoses)))
    if target.type == TargetType.Reef and isElevatorReefCoralL4Ready:
      targetAlignmentLocation = TargetAlignmentLocation.LeftL4 if targetAlignmentLocation == TargetAlignmentLocation.Left else TargetAlignmentLocation.RightL4
    return target.pose.transformBy(constants.Game.Field.Targets.kTargetAlignmentTransforms[target.type][targetAlignmentLocation])

  def hasVisionTarget(self) -> bool:
    for poseSensor in self._poseSensors:
      if poseSensor.hasTarget():
        return True
    return False

  def _updateTelemetry(self) -> None:
    self._robotPosePublisher.set(self._robotPose)

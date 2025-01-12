import math
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d, Translation2d, Pose3d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.pathfinding import PathConstraints
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import PID, MotorControllerType, SwerveModuleConstants, SwerveModuleConfig, SwerveModuleLocation, PoseSensorConfig, PoseSensorLocation, Alliance
from classes import Target, TargetType

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2025Reefscape)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)

    kTranslationSpeedMax: units.meters_per_second = 4.8
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi  # type: ignore

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      wheelBevelGearTeeth = 45,
      wheelSpurGearTeeth = 22,
      wheelBevelPinionTeeth = 15,
      drivingMotorPinionTeeth = 14,
      drivingMotorFreeSpeed = 5676,
      drivingMotorControllerType = MotorControllerType.SparkMax,
      drivingMotorCurrentLimit = 80,
      drivingMotorPID = PID(0.04, 0, 0),
      turningMotorCurrentLimit = 20,
      turningMotorPID = PID(1, 0, 0)
    )

    kSwerveModuleConfigs: tuple[SwerveModuleConfig, ...] = (
      SwerveModuleConfig(SwerveModuleLocation.FrontLeft, 2, 3, -math.pi / 2, Translation2d(kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.FrontRight, 4, 5, 0, Translation2d(kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearLeft, 6, 7, math.pi, Translation2d(-kWheelBase / 2, kTrackWidth / 2), _swerveModuleConstants),
      SwerveModuleConfig(SwerveModuleLocation.RearRight, 8, 9, math.pi / 2, Translation2d(-kWheelBase / 2, -kTrackWidth / 2), _swerveModuleConstants)
    )

    kDriveKinematics = SwerveDrive4Kinematics(*(c.translation for c in kSwerveModuleConfigs))

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPHolonomicDriveController(
      translation_constants = PIDConstants(5.0, 0, 0),
      rotation_constants = PIDConstants(5.0, 0, 0)
    )
    kPathFindingConstraints = PathConstraints(2.4, 1.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kDriftCorrectionControllerPID = PID(0.01, 0, 0)
    kDriftCorrectionPositionTolerance: float = 0.5
    kDriftCorrectionVelocityTolerance: float = 0.5

    kTargetAlignmentRotationPID = PID(0.075, 0, 0.001)
    kTargetAlignmentRotationPositionTolerance: float = 1.0
    kTargetAlignmentRotationVelocityTolerance: float = 1.0
    kTargetAlignmentRotationMaxSpeed: units.radians_per_second = units.degreesToRadians(720) #type: ignore

    kTargetAlignmentTranslationXPID = PID(0.3, 0, 0.003)
    kTargetAlignmentTranslationYPID = PID(0.6, 0, 0.006)

    kTargetAlignmentTranslationPositionTolerance: float = 0.05
    kTargetAlignmentTranslationVelocityTolerance: float = 0.05
    kTargetAlignmentTranslationMaxSpeed: units.meters_per_second = 2.0

    kTargetAlignmentCarpetFrictionCoeff: float = 0.2
    kTargetAlignmentHeadingAdjustment: units.degrees = 0.0
    kTargetAlignmentPoseRotationAdjustment: units.degrees = 180.0

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

  class Localization:
    kSingleTagStandardDeviations: tuple[float, ...] = (1.0, 1.0, 2.0)
    kMultiTagStandardDeviations: tuple[float, ...] = (0.5, 0.5, 1.0)
    kMaxPoseAmbiguity: units.percent = 0.2

class Sensors:
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    _poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    _fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        PoseSensorLocation.Front,
        Transform3d(
          Translation3d(units.inchesToMeters(9.62), units.inchesToMeters(4.12), units.inchesToMeters(21.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(24.2), units.degreesToRadians(0.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        PoseSensorLocation.Rear,
        Transform3d(
          Translation3d(units.inchesToMeters(5.49), units.inchesToMeters(0.0), units.inchesToMeters(20.60)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.2), units.degreesToRadians(-180.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        PoseSensorLocation.Left,
        Transform3d(
          Translation3d(units.inchesToMeters(8.24), units.inchesToMeters(12.40), units.inchesToMeters(17.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-29.4), units.degreesToRadians(90.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        PoseSensorLocation.Right,
        Transform3d(
          Translation3d(units.inchesToMeters(8.16), units.inchesToMeters(-12.375), units.inchesToMeters(17.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-21.2), units.degreesToRadians(-90.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      )
    )

  class Camera:
    kStreams: dict[str, str] = {
      "Rear": "http://10.28.81.6:1182/?action=stream",
      "Front": "http://10.28.81.6:1184/?action=stream",
      "Left": "http://10.28.81.7:1186/?action=stream",
      "Right": "http://10.28.81.7:1184/?action=stream",
      "Driver": "http://10.28.81.6:1188/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kAutoMoveTimeout: units.seconds = 4.0

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      _reefTargetTransform = Transform3d(
        units.inchesToMeters(24),
        units.inchesToMeters(0),
        units.inchesToMeters(0),
        Rotation3d()
      )

      _stationTargetTransform = Transform3d(
        units.inchesToMeters(24),
        units.inchesToMeters(0),
        units.inchesToMeters(0),
        Rotation3d()
      )

      _processorTargetTransform = Transform3d(
        units.inchesToMeters(24),
        units.inchesToMeters(0),
        units.inchesToMeters(0),
        Rotation3d()
      )

      _bargeTargetTransform = Transform3d(
        units.inchesToMeters(24),
        units.inchesToMeters(0),
        units.inchesToMeters(0),
        Rotation3d()
      )
      
      kTargets: list[Target] = [
        Target(1, TargetType.Station, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(1), _stationTargetTransform),
        Target(2, TargetType.Station, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(2), _stationTargetTransform),
        Target(3, TargetType.Processor, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(3), _processorTargetTransform),
        Target(4, TargetType.Barge, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(4), _bargeTargetTransform),
        Target(5, TargetType.Barge, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(5), _bargeTargetTransform),
        Target(6, TargetType.Reef, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(6), _reefTargetTransform),
        Target(7, TargetType.Reef, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(7), _reefTargetTransform),
        Target(8, TargetType.Reef, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(8), _reefTargetTransform),
        Target(9, TargetType.Reef, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(9), _reefTargetTransform),
        Target(10, TargetType.Reef, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(10), _reefTargetTransform),
        Target(11, TargetType.Reef, Alliance.Red, APRIL_TAG_FIELD_LAYOUT.getTagPose(11), _reefTargetTransform),
        Target(12, TargetType.Station, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(12), _stationTargetTransform),
        Target(13, TargetType.Station, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(13), _stationTargetTransform),
        Target(14, TargetType.Barge, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(14), _bargeTargetTransform),
        Target(15, TargetType.Barge, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(15), _bargeTargetTransform),
        Target(16, TargetType.Processor, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(16), _processorTargetTransform),
        Target(17, TargetType.Reef, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(17), _reefTargetTransform),
        Target(18, TargetType.Reef, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(18), _reefTargetTransform),
        Target(19, TargetType.Reef, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(19), _reefTargetTransform),
        Target(20, TargetType.Reef, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(20), _reefTargetTransform),
        Target(21, TargetType.Reef, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(21), _reefTargetTransform),
        Target(22, TargetType.Reef, Alliance.Blue, APRIL_TAG_FIELD_LAYOUT.getTagPose(22), _reefTargetTransform),
      ]


import math
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.pathfinding import PathConstraints
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import MotorControllerType, SwerveModuleConstants, SwerveModuleConfig, SwerveModuleLocation, PoseSensorConfig, PoseSensorLocation, PID, Tolerance, DriftCorrectionConstants, TargetAlignmentConstants, Alliance
from classes import Target, TargetType, TargetAlignmentLocation

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2025Reefscape)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)

    kTranslationSpeedMax: units.meters_per_second = 6.32
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi  # type: ignore

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      wheelBevelGearTeeth = 45,
      wheelSpurGearTeeth = 20,
      wheelBevelPinionTeeth = 15,
      drivingMotorPinionTeeth = 14,
      drivingMotorFreeSpeed = 6784,
      drivingMotorControllerType = MotorControllerType.SparkFlex,
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
    kPathPlannerController = PPHolonomicDriveController(PIDConstants(5.0, 0, 0), PIDConstants(5.0, 0, 0))
    kPathPlannerConstraints = PathConstraints(2.4, 1.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kDriftCorrectionConstants = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    kTargetAlignmentConstants = TargetAlignmentConstants(
      rotationPID = PID(0.075, 0, 0.001),
      rotationTolerance = Tolerance(1.0, 2.0),
      rotationSpeedMax = kRotationSpeedMax * 0.5,
      rotationHeadingModeOffset = 0.0,
      rotationTranslationModeOffset = 180.0,
      translationPID = PID(0.5, 0, 0.001),
      translationTolerance = Tolerance(0.05, 0.1),
      translationSpeedMax = kTranslationSpeedMax * 0.3
    )

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
      kTargets: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(1)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(2)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(3).toPose2d()): Target(TargetType.Processor, APRIL_TAG_FIELD_LAYOUT.getTagPose(3)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(4).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(4)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(5).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(5)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(6).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(6)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(7).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(7)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(8).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(8)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(9).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(9)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(10).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(10)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(11).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(12).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(12)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(13).toPose2d()): Target(TargetType.Station, APRIL_TAG_FIELD_LAYOUT.getTagPose(13)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(14).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(14)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(15).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(15)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(16).toPose2d()): Target(TargetType.Processor, APRIL_TAG_FIELD_LAYOUT.getTagPose(16)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(17).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(17)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(18).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(18)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(19).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(19)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(20).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(20)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(21).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(21)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(22).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(22))
        }
      }

      kTargetAlignmentTransforms: dict[TargetType, dict[TargetAlignmentLocation, Transform3d]] = {
        TargetType.Reef: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(24), units.inchesToMeters(-6.5), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(24), units.inchesToMeters(6.5), 0, Rotation3d())
        },
        TargetType.Station: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(12), units.inchesToMeters(-30), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(12), units.inchesToMeters(30), 0, Rotation3d())
        },
        TargetType.Processor: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d())
        },
        TargetType.Barge: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d())
        }
      }

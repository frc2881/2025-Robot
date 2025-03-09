import math
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
import wpilib
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController, PIDConstants
from pathplannerlib.pathfinding import PathConstraints
from photonlibpy.photonPoseEstimator import PoseStrategy
from rev import SparkLowLevel
from lib import logger, utils
from lib.classes import (
  Alliance, 
  PID, 
  Tolerance, 
  Position, 
  Range,
  Value,
  SwerveModuleConstants, 
  SwerveModuleConfig, 
  SwerveModuleLocation, 
  PoseSensorConstants,
  PoseSensorConfig, 
  DriftCorrectionConstants, 
  TargetAlignmentConstants, 
  PositionControlModuleConstants, 
  PositionControlModuleConfig 
)
from core.classes import (
  Target, 
  TargetType, 
  TargetAlignmentLocation, 
  TargetPosition, 
  TargetPositionType, 
  ElevatorPosition
)

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout(f'{ wpilib.getDeployDirectory() }/localization/2025-reefscape-andymark-nobarges.json')
# APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2025ReefscapeAndyMark)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kRobotLength: units.meters = units.inchesToMeters(36.0)
    kRobotWidth: units.meters = units.inchesToMeters(36.0)
    kTrackWidth: units.meters = units.inchesToMeters(26.0)
    kWheelBase: units.meters = units.inchesToMeters(26.0)

    kTranslationSpeedMax: units.meters_per_second = 5.74
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

    _swerveModuleConstants = SwerveModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      wheelBevelGearTeeth = 45,
      wheelSpurGearTeeth = 22,
      wheelBevelPinionTeeth = 15,
      drivingMotorPinionTeeth = 14,
      drivingMotorFreeSpeed = 6784,
      drivingMotorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      drivingMotorType = SparkLowLevel.MotorType.kBrushless,
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
    kPathPlannerConstraints = PathConstraints(3.6, 2.4, units.degreesToRadians(540), units.degreesToRadians(720))

    kDriftCorrectionConstants = DriftCorrectionConstants(
      rotationPID = PID(0.01, 0, 0), 
      rotationTolerance = Tolerance(0.5, 1.0)
    )

    kTargetAlignmentConstants = TargetAlignmentConstants(
      rotationPID = PID(0.075, 0, 0.001),
      rotationTolerance = Tolerance(0.5, 1.0),
      rotationSpeedMax = kRotationSpeedMax * 0.3, 
      rotationHeadingModeOffset = 0.0,
      rotationTranslationModeOffset = 180,
      translationPID = PID(5.0, 0, 0),
      translationTolerance = Tolerance(0.025, 0.05),
      translationSpeedMax = kTranslationSpeedMax * 0.3
    )

  class Elevator:
    _lowerStageModuleConstants = PositionControlModuleConstants(
      distancePerRotation = 0.5,
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(1.8, 0, 0.05),
      motorOutputRange = Range(-0.9, 1.0),
      motorMotionMaxVelocity = 300.0,
      motorMotionMaxAcceleration = 125.0,
      motorMotionAllowedClosedLoopError = 0.1,
      motorSoftLimitForward = 30.0,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.2
    )

    kLowerStageConfig = PositionControlModuleConfig("Elevator/LowerStage", 10, None, False, _lowerStageModuleConstants)
    kLowerStageHelperConfig = PositionControlModuleConfig("Elevator/LowerStage2", 20, 10, True, _lowerStageModuleConstants)

    kUpperStageConfig = PositionControlModuleConfig("Elevator/UpperStage", 11, None, False, PositionControlModuleConstants(
      distancePerRotation = 1.0,
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.01),
      # TODO: try adding velocityFF per REV example?
      motorOutputRange = Range(-0.4, 1.0),
      motorMotionMaxVelocity = 600.0,
      motorMotionMaxAcceleration = 200.0,
      motorMotionAllowedClosedLoopError = 0.1,
      motorSoftLimitForward = 28.0,
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.1
    ))

    kUpperStageSoftLimitBuffer: units.inches = 1.5
    kLowerStageReefCoralL4Position: units.inches = 15.0
    kInputLimit: units.percent = 0.5

  class Arm:
    kArmConfig = PositionControlModuleConfig("Arm", 12, None, True, PositionControlModuleConstants(
      distancePerRotation = 1.0,
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorReduction = 1.0 / 1.0,
      motorPID = PID(0.1, 0, 0.01),
      motorOutputRange = Range(-0.7, 1.0),
      motorMotionMaxVelocity = 200,
      motorMotionMaxAcceleration = 300.0,
      motorMotionAllowedClosedLoopError = 0.1,
      motorSoftLimitForward = 69.8,
      motorSoftLimitReverse = 1.5,
      motorResetSpeed = 0.2
    ))

    kInputLimit: units.percent = 0.6
    kCoralStageReadyPosition: float = 11.2

  class Wrist:
    kMotorCANId: int = 13
    kMotorCurrentLimit: int = 20
    kMotorUpSpeed: units.percent = 0.7
    kMotorDownSpeed: units.percent = 0.3
    kMotorHoldUpSpeed: units.percent = 0.01
    kMotorHoldDownSpeed: units.percent = 0.05
    kSetPositionTimeout: units.seconds = 0.8

  class Hand:
    kGripperMotorCANId: int = 14
    kGripperMotorCurrentLimit: int = 30
    kGripperMotorCurrentTrigger: int = 25
    kGripperMotorSpeed: units.percent = 1.0
    kGripperReleaseTimeout: units.seconds = 0.75

    kSuctionMotorCANId: int = 15
    kSuctionMotorCurrentLimit: int = 20
    kSuctionMotorCurrentTrigger: int = 5
    kSuctionMotorSpeed: units.percent = 0.5
    kSuctionReleaseTimeout: units.seconds = 3.0

  class Shield:
    kServoChannel: int = 9
    kServoSetPositionTimeout: units.seconds = 1.0
    kPositionOpen: float = 0.0
    kPositionClosed: float = 1.0

class Services:
  class Localization:
    kStateStandardDeviations: tuple[float, float, float] = (0.04, 0.04, units.degreesToRadians(1))
    kVisionStandardDeviations: tuple[float, float, float] = (0.4, 0.4, units.degreesToRadians(4))
    kVisionMaxPoseAmbiguity: units.percent = 0.2

class Sensors: 
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    _poseSensorConstants = PoseSensorConstants(
      aprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT,
      poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
      fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    )

    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        "FrontLeft",
        Transform3d(
          Translation3d(units.inchesToMeters(-4.0972), units.inchesToMeters(8.125), units.inchesToMeters(39.3044)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-30.0), units.degreesToRadians(0))
        ), _poseSensorConstants
      ),
      PoseSensorConfig(
        "FrontRight",
        Transform3d(
          Translation3d(units.inchesToMeters(-4.1391), units.inchesToMeters(-8.125), units.inchesToMeters(38.3683)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(32.0), units.degreesToRadians(0))
        ), _poseSensorConstants
      ),
      PoseSensorConfig(
        "RearLeft",
        Transform3d(
          Translation3d(units.inchesToMeters(-8.8236), units.inchesToMeters(7.2958), units.inchesToMeters(36.1419)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(20.0), units.degreesToRadians(165.0))
        ), _poseSensorConstants
      ),
      PoseSensorConfig(
        "RearRight",
        Transform3d(
          Translation3d(units.inchesToMeters(-8.8236), units.inchesToMeters(-7.2958), units.inchesToMeters(36.1515)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(20.0), units.degreesToRadians(-165.0))
        ), _poseSensorConstants
      )
    )

  class Camera:
    kStreams: dict[str, str] = {
      "FrontRight": "http://10.28.81.6:1184/?action=stream",
      "FrontLeft": "http://10.28.81.7:1182/?action=stream",
      "RearRight": "http://10.28.81.6:1182/?action=stream",
      "RearLeft": "http://10.28.81.7:1186/?action=stream",
      "Driver": "http://10.28.81.6:1184/?action=stream",
      "Internal": "http://10.28.81.7:1184/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kTargetAlignmentTimeout: units.seconds = 2.0 
    kAutoMoveTimeout: units.seconds = 4.0

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kTargets: dict[Alliance, dict[int, Target]] = {
        Alliance.Red: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(1).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(1)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(2).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(2)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(3).toPose2d()): Target(TargetType.AlgaeProcessor, APRIL_TAG_FIELD_LAYOUT.getTagPose(3)),
          # utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(4).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(4)),
          # utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(5).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(5)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(6).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(6)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(7).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(7)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(8).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(8)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(9).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(9)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(10).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(10)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(11).toPose2d()): Target(TargetType.Reef, APRIL_TAG_FIELD_LAYOUT.getTagPose(11))
        },
        Alliance.Blue: {
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(12).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(12)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(13).toPose2d()): Target(TargetType.CoralStation, APRIL_TAG_FIELD_LAYOUT.getTagPose(13)),
          # utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(14).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(14)),
          # utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(15).toPose2d()): Target(TargetType.Barge, APRIL_TAG_FIELD_LAYOUT.getTagPose(15)),
          utils.getTargetHash(APRIL_TAG_FIELD_LAYOUT.getTagPose(16).toPose2d()): Target(TargetType.AlgaeProcessor, APRIL_TAG_FIELD_LAYOUT.getTagPose(16)),
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
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(36), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(22), units.inchesToMeters(-6.5), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(22), units.inchesToMeters(6.5), 0, Rotation3d()),
          TargetAlignmentLocation.LeftL4: Transform3d(units.inchesToMeters(23.5), units.inchesToMeters(-6.5), 0, Rotation3d()),
          TargetAlignmentLocation.RightL4: Transform3d(units.inchesToMeters(23.5), units.inchesToMeters(6.5), 0, Rotation3d())
        },
        TargetType.CoralStation: {
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(23), units.inchesToMeters(0.0-2.0), 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(23), units.inchesToMeters(-24.0), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(23), units.inchesToMeters(24.0), 0, Rotation3d())
        }
      }

      kTargetPositions: dict[TargetPositionType, TargetPosition] = {
        TargetPositionType.ReefCoralL4: TargetPosition(ElevatorPosition(28.5, Value.max), 7.5, Position.Down),
        TargetPositionType.ReefCoralL3: TargetPosition(ElevatorPosition(Value.min, Value.max), 3.6, Position.Down),
        TargetPositionType.ReefCoralL2: TargetPosition(ElevatorPosition(Value.min, 11.30), 1.0, Position.Down),
        TargetPositionType.ReefCoralL1: TargetPosition(ElevatorPosition(Value.min, 23.0), 30, Position.Up),
        TargetPositionType.ReefAlgaeL3: TargetPosition(ElevatorPosition(6.5, Value.max), 19.3, Position.Down),
        TargetPositionType.ReefAlgaeL2: TargetPosition(ElevatorPosition(6.5, 19), 24.0, Position.Down),
        TargetPositionType.CoralStation: TargetPosition(ElevatorPosition(Value.min, Value.min), Value.min, Position.Up),
        TargetPositionType.CageDeepClimb: TargetPosition(ElevatorPosition(7.0, Value.max), Value.max, Position.Up)
      }

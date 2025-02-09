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
from rev import SparkLowLevel
from lib import logger, utils
from lib.classes import Alliance, PID, Tolerance, SwerveModuleConstants, SwerveModuleConfig, SwerveModuleLocation, PoseSensorConfig, DriftCorrectionConstants, TargetAlignmentConstants, PositionControlModuleConstants, PositionControlModuleConfig
from core.classes import Target, TargetType, TargetAlignmentLocation, ReefLevel, ElevatorPositions

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2025Reefscape)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(26)
    kWheelBase: units.meters = units.inchesToMeters(26)

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
      rotationTolerance = Tolerance(1.0, 2.0),
      rotationSpeedMax = kRotationSpeedMax * 0.5, 
      rotationHeadingModeOffset = 0.0,
      rotationTranslationModeOffset = 180,
      translationPID = PID(5.0, 0, 0),
      translationTolerance = Tolerance(0.025, 0.05),
      translationSpeedMax = kTranslationSpeedMax * 0.5
    )

  class Elevator:
    kLowerStageModuleConfig = PositionControlModuleConfig("Elevator/Lower", 10, None, False, PositionControlModuleConstants(
        motorTravelDistance = 0.5,
        motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
        motorType = SparkLowLevel.MotorType.kBrushless,
        motorCurrentLimit = 80,
        motorReduction = 3.0,
        motorPID = PID(0.1, 0, 0.01),
        motorMotionMaxVelocityRate = 33.0,
        motorMotionMaxAccelerationRate = 66.0,
        allowedClosedLoopError = 0.1,
        motorSoftLimitForward = 28.9, # TODO: Update Elevator soft limits
        motorSoftLimitReverse = 0.5,
        motorResetSpeed = 0.1
    ))
    kUpperStageModuleConfig = PositionControlModuleConfig("Elevator/Upper", 11, None, False, PositionControlModuleConstants(
      motorTravelDistance = 1.0,
      motorControllerType = SparkLowLevel.SparkModel.kSparkFlex,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 80,
      motorReduction = 1.0,
      motorPID = PID(0.1, 0, 0.01),
      motorMotionMaxVelocityRate = 33.0,
      motorMotionMaxAccelerationRate = 66.0,
      allowedClosedLoopError = 0.1,
      motorSoftLimitForward = 20.0, # 30.0, # TODO: Update Elevator soft limits
      motorSoftLimitReverse = 0.5,
      motorResetSpeed = 0.12
    ))

    kPositionsAlignmentPositionTolerance: float = 0.05

    kInputLimit: units.percent = 0.5

    kElevatorScoringPositions: dict[ReefLevel, ElevatorPositions] = {
      ReefLevel.L1: ElevatorPositions(0, 0),
      ReefLevel.L2: ElevatorPositions(0, 0),
      ReefLevel.L3: ElevatorPositions(0, 0),
      ReefLevel.L4: ElevatorPositions(0, 0)
    }

  class Arm:
    kArmPositonControlModuleConfig = PositionControlModuleConfig("Arm/Motor", 12, None, True, PositionControlModuleConstants(
      motorTravelDistance = 1.0,
      motorControllerType = SparkLowLevel.SparkModel.kSparkMax,
      motorType = SparkLowLevel.MotorType.kBrushless,
      motorCurrentLimit = 60,
      motorReduction = 1.0,
      motorPID = PID(0.1, 0, 0.01),
      motorMotionMaxVelocityRate = 33.0,
      motorMotionMaxAccelerationRate = 66.0,
      allowedClosedLoopError = 0.1,
      motorSoftLimitForward = 48, # TODO: Update Elevator soft limits
      motorSoftLimitReverse = 0,
      motorResetSpeed = 0.1
    ))

    kPositionAlignmentPositionTolerance: float = 0.5  

    kInputLimit: units.percent = 0.5

    kArmScoringPositions: dict[ReefLevel, float] = {
      ReefLevel.L1: 0.0,
      ReefLevel.L2: 0.0,
      ReefLevel.L3: 0.0,
      ReefLevel.L4: 0.0
    }

  class Wrist:
    kWristMotorCANId: int = 13
    kWristMotorCurrentLimit: int = 20
    kWristInputLimit: units.percent = 0.2
    kWristMoveSpeedUp: units.percent = 0.3
    kWristMoveSpeedDown: units.percent = 0.2
    kWristMaxCurrent: int = 20 # TODO: Update WristMaxCurrent

  class Hand:
    kGripperMotorCANId: int = 14
    kGripperMotorCurrentLimit: int = 20
    kGripperMaxCurrent: int = 17
    kGripperIntakeSpeed: units.percent = 1.0
    kGripperHoldSpeed: units.percent = 0.2 # TODO: Tune with real mechanism
    kGripperEjectSpeed: units.percent = -0.75
    kSuctionMotorCANId: int = 15
    kSuctionMotorCurrentLimit: int = 60
    kSuctionMotorSpeed: units.percent = 0.2
    kSuctionMaxCurrent: int = 60 # TODO: Update SuctionMaxCurrent
    kSuctionSolenoidPortId: int = 0

class Services:
  class Localization:
    kStateStandardDeviations: tuple[float, float, float] = (0.1, 0.1, units.degreesToRadians(5))
    kVisionMultiTagStandardDeviations: tuple[float, float, float] = (0.2, 0.2, units.degreesToRadians(10))
    kVisionDefaultStandardDeviations: tuple[float, float, float] = (0.5, 0.5, units.degreesToRadians(25)) # TODO: update based on testing if single tag and multi tag should be equal weight
    kVisionMaxPoseAmbiguity: units.percent = 0.2

class Sensors: 
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    _poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    _fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    
    # TODO: apply actual precise measurements for camera transforms after mounting to the robot (rought placehlders for now)
    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        "FrontRight",
        Transform3d(
          Translation3d(units.inchesToMeters(-4), units.inchesToMeters(-7.875), units.inchesToMeters(38.375)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(32.1), units.degreesToRadians(0.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        "FrontLeft",
        Transform3d(
          Translation3d(units.inchesToMeters(-3.75), units.inchesToMeters(7.75), units.inchesToMeters(39.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-29.4), units.degreesToRadians(0.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        "RearRight",
        Transform3d(
          Translation3d(units.inchesToMeters(-9), units.inchesToMeters(-7.25), units.inchesToMeters(36.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(5.5), units.degreesToRadians(-180.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        "RearLeft",
        Transform3d(
          Translation3d(units.inchesToMeters(-8.75), units.inchesToMeters(7.25), units.inchesToMeters(36.625)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(5.0), units.degreesToRadians(-180.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      )
    )

  class Camera:
    kStreams: dict[str, str] = {
      "FrontRight": "http://10.28.81.6:1182/?action=stream",
      "FrontLeft": "http://10.28.81.7:1182/?action=stream",
      "RearRight": "http://10.28.81.6:1184/?action=stream",
      "RearLeft": "http://10.28.81.7:1184/?action=stream",
      "Driver": "http://10.28.81.6:1182/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kAutoMoveTimeout: units.seconds = 4.0
    kAutoTargetAlignmentTimeout: units.seconds = 2.0

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

      # TODO: update target alignment transforms for real world scoring and pickup once robot is operational for testing
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
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(12), units.inchesToMeters(-24), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(12), units.inchesToMeters(24), 0, Rotation3d())
        },
        TargetType.Processor: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(24), 0, 0, Rotation3d())
        },
        TargetType.Barge: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d())
        }
      }

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
from lib.classes import Alliance, PID, Tolerance, MotorControllerType, SwerveModuleConstants, SwerveModuleConfig, SwerveModuleLocation, PoseSensorConfig, ObjectSensorConfig, DriftCorrectionConstants, TargetAlignmentConstants, PositionControlModuleConstants, PositionControlModuleConfig
from core.classes import Target, TargetType, TargetAlignmentLocation

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2025Reefscape)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)

    kTranslationSpeedMax: units.meters_per_second = 6.32
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi # type: ignore

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
    _leadscrewModuleLowerConstants = PositionControlModuleConstants(
        motorTravelDistance = 0.5,
        motorControllerType = MotorControllerType.SparkFlex,
        motorCurrentLimit = 60,
        motorReduction = 3.0,
        motorPID = PID(0.1, 0, 0.01),
        motorMotionMaxVelocityRate = 33.0,
        motorMotionMaxAccelerationRate = 66.0,
        allowedClosedLoopError = 0.1,
        motorSoftLimitForward = 22.50, # TODO: Update Elevator soft limits
        motorSoftLimitReverse = 0,
        motorResetSpeed = 0.1 
    )

    _leadscrewModuleUpperConstants = PositionControlModuleConstants(
      motorTravelDistance = 1.0,
      motorControllerType = MotorControllerType.SparkFlex,
      motorCurrentLimit = 60,
      motorReduction = 1.0,
      motorPID = PID(0.1, 0, 0.01),
      motorMotionMaxVelocityRate = 33.0,
      motorMotionMaxAccelerationRate = 66.0,
      allowedClosedLoopError = 0.1,
      motorSoftLimitForward = 22.50, # TODO: Update Elevator soft limits
      motorSoftLimitReverse = 0,
      motorResetSpeed = 0.1
    )

    kLeadScrewModuleConfigLower = PositionControlModuleConfig("Elevator/Leadscrews/Lower", 10, None, _leadscrewModuleLowerConstants)
    kLeadScrewModuleConfigUpper = PositionControlModuleConfig("Elevator/Leadscrews/Upper", 11, None, _leadscrewModuleUpperConstants)

    kHeightAlignmentPositionTolerance: float = 0.05

    kInputLimit: units.percent = 0.5

  class Arm:
    _armPositionControlModuleUpperConstants = PositionControlModuleConstants(
      motorTravelDistance = 1.0,
      motorControllerType = MotorControllerType.SparkFlex,
      motorCurrentLimit = 60,
      motorReduction = 1.0,
      motorPID = PID(0.1, 0, 0.01),
      motorMotionMaxVelocityRate = 33.0,
      motorMotionMaxAccelerationRate = 66.0,
      allowedClosedLoopError = 0.1,
      motorSoftLimitForward = 22.50, # TODO: Update Elevator soft limits
      motorSoftLimitReverse = 0,
      motorResetSpeed = 0.1
    )

    kArmPositonControlModuleConfig = PositionControlModuleConfig("Arm/Motor", 12, None, _armPositionControlModuleUpperConstants)

    kPositionAlignmentPositionTolerance: float = 0.5  

    kInputLimit: units.percent = 0.5

  class Wrist:
    pass

  class Hand:
    pass

  class Intake:
    _leadscrewModuleConstants = PositionControlModuleConstants(
      motorTravelDistance = 0.5,
      motorControllerType = MotorControllerType.SparkMax,
      motorCurrentLimit = 60,
      motorReduction = 3.0,
      motorPID = PID(0.1, 0, 0.01),
      motorMotionMaxVelocityRate = 33.0,
      motorMotionMaxAccelerationRate = 66.0,
      allowedClosedLoopError = 0.1,
      motorSoftLimitForward = 22.50, # TODO: Update Intake soft limits
      motorSoftLimitReverse = 0,
      motorResetSpeed = 0.1
    )

    kStartingPosition: float = 0.0 # TODO: Update Intake Positions
    kDefaultPosition: float = 0.0 # TODO: Update Intake Positions
    kCoralIntakePosition: float = 0.0 # TODO: Update Intake Positions
    kAlgaeIntakePosition:float  = 0.0 # TODO: Update Intake Positions

    kLeadScrewModuleConfigRight = PositionControlModuleConfig("Launcher/Arm/Leadscrews/Right", 16, 17, _leadscrewModuleConstants)
    kLeadScrewModuleConfigLeft = PositionControlModuleConfig("Launcher/Arm/Leadscrews/Left", 17, None, _leadscrewModuleConstants)

    kRollerMotorCANId: int = 18
    kRollerMotorCurrentLimit: int = 60

    kTargetAlignmentPositionTolerance: float = 0.05

    kInputLimit: units.percent = 0.5


class Services:
  class Localization:
    kStateStandardDeviations: tuple[float, float, float] = (0.1, 0.1, units.degreesToRadians(5))
    kVisionMultiTagStandardDeviations: tuple[float, float, float] = (0.2, 0.2, units.degreesToRadians(10))
    kVisionDefaultStandardDeviations: tuple[float, float, float] = (0.5, 0.5, units.degreesToRadians(25))
    kVisionMaxPoseAmbiguity: units.percent = 0.2

class Sensors: 
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    _poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    _fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    
    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        "Front",
        Transform3d(
          Translation3d(units.inchesToMeters(9.62), units.inchesToMeters(4.12), units.inchesToMeters(21.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(24.2), units.degreesToRadians(0.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        "Rear",
        Transform3d(
          Translation3d(units.inchesToMeters(5.49), units.inchesToMeters(0.0), units.inchesToMeters(20.60)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-23.2), units.degreesToRadians(-180.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        "Left",
        Transform3d(
          Translation3d(units.inchesToMeters(8.24), units.inchesToMeters(12.40), units.inchesToMeters(17.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-29.4), units.degreesToRadians(90.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
      PoseSensorConfig(
        "Right",
        Transform3d(
          Translation3d(units.inchesToMeters(8.16), units.inchesToMeters(-12.375), units.inchesToMeters(17.25)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-21.2), units.degreesToRadians(-90.0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      )
    )

  class Object:
    kObjectSensorConfig = ObjectSensorConfig(
      "Front",
      Transform3d(
        Translation3d(units.inchesToMeters(9.62), units.inchesToMeters(4.12), units.inchesToMeters(19.25)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(24.2), units.degreesToRadians(0.0))
      )
    )

  class Camera:
    kStreams: dict[str, str] = {
      "Rear": "http://10.28.81.6:1182/?action=stream",
      "Front": "http://10.28.81.6:1184/?action=stream",
      "Left": "http://10.28.81.7:1186/?action=stream",
      "Right": "http://10.28.81.7:1184/?action=stream",
      "Driver": "http://10.28.81.6:1186/?action=stream"
    }

  class Distance:
    class Intake:
      kSensorName = "Intake"
      kMinTargetDistance: units.millimeters = 1
      kMaxTargetDistance: units.millimeters = 320


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
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(0), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(0), units.inchesToMeters(-30), 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(0), units.inchesToMeters(30), 0, Rotation3d())
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
        },
        TargetType.Object: {
          TargetAlignmentLocation.Default: Transform3d(),
          TargetAlignmentLocation.Center: Transform3d(units.inchesToMeters(6), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Left: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d()),
          TargetAlignmentLocation.Right: Transform3d(units.inchesToMeters(12), 0, 0, Rotation3d())
        }
      }

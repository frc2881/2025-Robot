from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d
from lib.classes import Position

class TargetType(Enum):
  Reef = auto()
  CoralStation = auto()

class TargetPositionType(Enum):
  ReefCoralL4 = auto()
  ReefCoralL3 = auto()
  ReefCoralL2 = auto()
  ReefCoralL1 = auto()
  ReefAlgaeL3 = auto()
  ReefAlgaeL2 = auto()
  CoralStation = auto()
  IntakeReady = auto()
  IntakeHandoff = auto()
  IntakeLift = auto()
  CageDeepClimb = auto()

class TargetAlignmentLocation(Enum):
  Center = auto()
  Left = auto()
  LeftL4 = auto()
  Right = auto()
  RightL4 = auto()

@dataclass(frozen=True, slots=True)
class Target():
  type: TargetType
  pose: Pose3d

@dataclass(frozen=False, slots=True)
class TargetAlignmentInfo:
  pose: Pose2d
  distance: units.meters
  heading: units.degrees
  pitch: units.degrees

@dataclass(frozen=False, slots=True)
class ElevatorPosition:
  lowerStage: units.inches
  upperStage: units.inches

class ElevatorStage(Enum):
  Both = auto()
  Upper = auto()
  Lower = auto()

@dataclass(frozen=False, slots=True)
class TargetPosition:
  elevator: ElevatorPosition
  arm: units.inches
  wrist: Position

class LightsMode(Enum):
  Default = auto()
  RobotNotConnected = auto()
  RobotNotReset = auto()
  VisionNotReady = auto()
  IntakeReady = auto()
  ScoreReady = auto()
  ClimbReady = auto()

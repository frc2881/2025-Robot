from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d

class GamePiece(Enum):
  Coral = auto()
  Algae = auto()

class TargetType(Enum):
  Default = auto()
  Reef = auto()
  CoralStation = auto()
  AlgaeProcessor = auto()
  Barge = auto()

class TargetPositionType(Enum):
  Default = auto()
  Start = auto()
  ReefCoralL4 = auto()
  ReefAlgaeL3 = auto()
  ReefCoralL3 = auto()
  ReefAlgaeL2 = auto()
  ReefCoralL2 = auto()
  ReefCoralL1 = auto()
  CoralStation = auto()
  AlgaeProcessor = auto()
  Barge = auto()
  CageEntry = auto()
  CageClimb = auto()

class TargetAlignmentLocation(Enum):
  Default = auto()
  Center = auto()
  Left = auto()
  Right = auto()

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

class WristPosition(Enum):
  Unknown = auto()
  Up = auto()
  Down = auto()

@dataclass(frozen=False, slots=True)
class TargetPosition:
  elevator: ElevatorPosition
  arm: units.inches
  wrist: WristPosition

class LightsMode(Enum):
  Default = auto()
  RobotNotReady = auto()
  VisionNotReady = auto()

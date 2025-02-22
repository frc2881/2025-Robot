from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d
from lib.classes import Position

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
  CoralStation = auto()
  ReefCoralL4Ready = auto()
  ReefCoralL4Score = auto()
  ReefCoralL3Ready = auto()
  ReefCoralL3Score = auto()
  ReefCoralL2Ready = auto()
  ReefCoralL2Score = auto()
  ReefCoralL1Ready = auto()
  ReefCoralL1Score = auto()
  ReefAlgaeL3 = auto()
  ReefAlgaeL2 = auto()
  AlgaeProcessor = auto()
  Barge = auto()
  CageEntry = auto()

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
  RobotNotReady = auto()
  VisionNotReady = auto()

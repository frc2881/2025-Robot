from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Pose2d, Pose3d

class TargetType(Enum):
  Default = auto()
  Reef = auto()
  Station = auto()
  Processor = auto()
  Barge = auto()
  Object = auto()

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

from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Pose3d, Transform3d
from lib.classes import Alliance

class TargetType(Enum):
  Reef = auto()
  Station = auto()
  Processor = auto()
  Barge = auto()

@dataclass(frozen=True, slots=True)
class Target:
  id: int
  type: TargetType
  alliance: Alliance
  pose: Pose3d
  transform: Transform3d
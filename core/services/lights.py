from typing import Callable
from wpilib import DriverStation
from lib.classes import RobotState, RobotMode
from lib.controllers.lights import Lights as LightsController
from lib import logger, utils
from core.classes import LightsMode

class Lights():
  def __init__(
      self,
      hasAllZeroResets: Callable[[], bool],
      hasValidVisionTarget: Callable[[], bool],
      isRobotAlignedForScoring: Callable[[], bool],
      isGripperHolding: Callable[[], bool]
    ) -> None:
    super().__init__()
    self._hasAllZeroResets = hasAllZeroResets
    self._hasValidVisionTarget = hasValidVisionTarget
    self._isGripperHolding = isGripperHolding
    self._isRobotAlignedForScoring = isRobotAlignedForScoring

    self._lightsController = LightsController()

    utils.addRobotPeriodic(self._periodic)

  def _periodic(self) -> None:
    self._updateLights()

  def _updateLights(self) -> None:
    if not DriverStation.isDSAttached():
      self._lightsController.setMode(LightsMode.RobotNotConnected)
      return
    
    if utils.getRobotState() == RobotState.Disabled:
      if not utils.isCompetitionMode() and not self._hasAllZeroResets():
        self._lightsController.setMode(LightsMode.RobotNotReset)
        return
      if utils.isCompetitionMode() and not self._hasValidVisionTarget():
        self._lightsController.setMode(LightsMode.VisionNotReady)
        return
    else:
      if utils.getRobotMode() == RobotMode.Teleop and utils.isValueInRange(utils.getMatchTime(), 0.02, 15):
        self._lightsController.setMode(LightsMode.ClimbReady)
        return
      if self._isRobotAlignedForScoring():
        self._lightsController.setMode(LightsMode.ScoreReady)
        return
      if self._isGripperHolding():
        self._lightsController.setMode(LightsMode.IntakeReady)
        return
      
    self._lightsController.setMode(LightsMode.Default)
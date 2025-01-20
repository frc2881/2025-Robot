from typing import TYPE_CHECKING
from commands2 import Command, cmd
from wpilib import RobotBase
from lib import logger, utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern, TargetAlignmentMode
if TYPE_CHECKING: from core.robot import RobotCore
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants

class GameCommands:
  def __init__(
      self,
      robot: "RobotCore"
    ) -> None:
    self._robot = robot

  def alignRobotToTargetCommand(self, targetAlignmentMode: TargetAlignmentMode, targetAlignmentLocation: TargetAlignmentLocation = TargetAlignmentLocation.Default, targetType: TargetType = TargetType.Default) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self._robot.driveSubsystem.alignToTargetCommand(
          self._robot.localizationService.getRobotPose, 
          self._robot.localizationService.getTargetPose,
          targetAlignmentMode,
          targetAlignmentLocation,
          targetType
        ),
        self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short),
        cmd.sequence(
          cmd.waitUntil(self._robot.driveSubsystem.isAlignedToTarget),
          self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
        )
      )
    ).withName("GameCommands:AlignRobotToTarget")

  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self._robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self._robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")
#! python3

from commands2 import CommandScheduler, cmd, TimedCommandRobot
from lib import logger, telemetry, utils
from lib.classes import RobotMode
from core.robot import RobotCore

class Robot(TimedCommandRobot):
  def __init__(self) -> None:
    TimedCommandRobot.__init__(self)
    utils.setRobotInstance(self)
    logger.start()
    telemetry.start()
    self._autoCommand = cmd.none()
    self._robotCore = RobotCore()

  def robotPeriodic(self) -> None:
    try:
      CommandScheduler.getInstance().run()
    except:
      CommandScheduler.getInstance().cancelAll()
      self._robotCore.resetRobot()
      logger.exception()

  def disabledInit(self) -> None:
    logger.mode(RobotMode.Disabled)

  def disabledPeriodic(self) -> None:
    pass

  def autonomousInit(self) -> None:
    logger.mode(RobotMode.Auto)
    self._robotCore.autoInit()
    self._autoCommand = self._robotCore.getAutoCommand()
    if self._autoCommand is not None:
      self._autoCommand.schedule()

  def autonomousPeriodic(self) -> None:
    pass

  def autonomousExit(self):
    self._robotCore.autoExit()

  def teleopInit(self) -> None:
    logger.mode(RobotMode.Teleop)
    if self._autoCommand is not None:
      self._autoCommand.cancel()
    self._robotCore.teleopInit()

  def teleopPeriodic(self) -> None:
    pass

  def testInit(self) -> None:
    logger.mode(RobotMode.Test)
    CommandScheduler.getInstance().cancelAll()
    self._robotCore.testInit()

  def testPeriodic(self) -> None:
    pass

  def _simulationInit(self) -> None:
    pass

  def _simulationPeriodic(self) -> None:
    pass

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
    self._auto = cmd.none()
    self._robot = RobotCore()

  def robotPeriodic(self) -> None:
    try:
      CommandScheduler.getInstance().run()
    except:
      CommandScheduler.getInstance().cancelAll()
      self._robot.reset()
      logger.exception()

  def disabledInit(self) -> None:
    logger.mode(RobotMode.Disabled)
    self._robot.disabledInit()

  def disabledPeriodic(self) -> None:
    pass

  def autonomousInit(self) -> None:
    logger.mode(RobotMode.Auto)
    self._robot.autoInit()
    self._auto = self._robot.auto.get()
    if self._auto is not None:
      self._auto.schedule()

  def autonomousPeriodic(self) -> None:
    pass

  def autonomousExit(self):
    self._robot.autoExit()

  def teleopInit(self) -> None:
    logger.mode(RobotMode.Teleop)
    if self._auto is not None:
      self._auto.cancel()
    self._robot.teleopInit()

  def teleopPeriodic(self) -> None:
    pass

  def testInit(self) -> None:
    logger.mode(RobotMode.Test)
    CommandScheduler.getInstance().cancelAll()
    self._robot.testInit()

  def testPeriodic(self) -> None:
    pass

  def _simulationInit(self) -> None:
    self._robot.simulationInit()

  def _simulationPeriodic(self) -> None:
    pass

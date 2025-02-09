from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.game_controller import GameController
from lib.sensors.gyro_sensor_navx2 import GyroSensor_NAVX2
from lib.sensors.pose_sensor import PoseSensor
from lib.sensors.distance_sensor import DistanceSensor
from core.commands.auto import AutoCommands
from core.commands.game import GameCommands
from core.subsystems.drive import DriveSubsystem
from core.subsystems.elevator import ElevatorSubsystem
from core.subsystems.arm import ArmSubsystem
from core.subsystems.wrist import WristSubsystem
from core.subsystems.hand import HandSubsystem
from core.services.localization import LocalizationService
from core.classes import TargetAlignmentLocation, TargetType
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initControllers()
    self._initCommands()
    self._initTriggers()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyroSensor = GyroSensor_NAVX2(constants.Sensors.Gyro.NAVX2.kComType)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.kPoseSensorConfigs)
    SmartDashboard.putString("Robot/Sensors/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))

  def _initSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(self.gyroSensor.getHeading)
    self.elevatorSubsystem = ElevatorSubsystem()
    self.armSubsystem = ArmSubsystem()
    self.wristSubsystem = WristSubsystem()
    self.handSubsystem = HandSubsystem()
    
  def _initServices(self) -> None:
    self.localizationService = LocalizationService(self.gyroSensor.getRotation, self.driveSubsystem.getModulePositions, self.poseSensors)
    
  def _initControllers(self) -> None:
    self.driverController = GameController(constants.Controllers.kDriverControllerPort, constants.Controllers.kInputDeadband)
    self.operatorController = GameController(constants.Controllers.kOperatorControllerPort, constants.Controllers.kInputDeadband)
    DriverStation.silenceJoystickConnectionWarning(True)

  def _initCommands(self) -> None:
    self.gameCommands = GameCommands(self)
    self.autoCommands = AutoCommands(self)

  def _initTriggers(self) -> None:
    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveCommand(
        self.driverController.getLeftY,
        self.driverController.getLeftX,
        self.driverController.getRightX
    ))
    self.driverController.rightStick().and_((self.driverController.rightBumper().or_(self.driverController.leftBumper())).negate()).whileTrue(self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Center))
    self.driverController.rightStick().and_(self.driverController.rightBumper()).whileTrue(self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Right))
    self.driverController.rightStick().and_(self.driverController.leftBumper()).whileTrue(self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Left))
    self.driverController.leftStick().whileTrue(self.driveSubsystem.lockCommand())
    # self.driverController.rightTrigger().whileTrue(cmd.none())
    # self.driverController.rightBumper().whileTrue(cmd.none())
    # self.driverController.leftTrigger().whileTrue(cmd.none())
    # self.driverController.leftBumper().whileTrue(cmd.none())
    self.driverController.povUp().whileTrue(self.autoCommands.moveToStartingPosition(2))
    # self.driverController.povDown().whileTrue(cmd.none())
    self.driverController.povLeft().whileTrue(self.autoCommands.moveToStartingPosition(1))
    self.driverController.povRight().whileTrue(self.autoCommands.moveToStartingPosition(3))
    # self.driverController.a().whileTrue(cmd.none())
    # self.driverController.b().whileTrue(cmd.none())
    # self.driverController.y().whileTrue(cmd.none())
    # self.driverController.x().whileTrue(cmd.none())
    self.driverController.start().onTrue(self.gyroSensor.calibrateCommand())
    self.driverController.back().onTrue(self.gyroSensor.resetCommand())

    self.elevatorSubsystem.setDefaultCommand(
      self.elevatorSubsystem.runCommand(
        self.operatorController.getLeftY
    ))

    self.armSubsystem.setDefaultCommand(
      self.armSubsystem.runCommand(
        self.operatorController.getRightY
    ))

    self.operatorController.rightTrigger().whileTrue(self.wristSubsystem.moveDownCommand())
    # self.operatorController.rightBumper().whileTrue(cmd.none())
    self.operatorController.leftTrigger().whileTrue(self.wristSubsystem.moveUpCommand())
    # self.operatorController.leftBumper().whileTrue(cmd.none())
    # self.operatorController.povUp().whileTrue(cmd.none())
    # self.operatorController.povDown().whileTrue(cmd.none())
    # self.operatorController.povLeft().whileTrue(cmd.none())
    # self.operatorController.povRight().whileTrue(cmd.none())
    # self.operatorController.a().whileTrue(cmd.none())
    # self.operatorController.b().whileTrue(cmd.none())
    # self.operatorController.y().whileTrue(cmd.none())
    self.operatorController.x().whileTrue(self.handSubsystem.runRollerCommand(1.0))
    self.operatorController.start().whileTrue(self.armSubsystem.resetToZeroCommand())
    self.operatorController.back().and_(self.operatorController.povLeft()).whileTrue(self.elevatorSubsystem.resetToZeroCommandLower())
    self.operatorController.back().and_(self.operatorController.povUp()).whileTrue(self.elevatorSubsystem.resetToZeroCommandUpper())
    self.operatorController.back().and_(self.operatorController.povRight()).whileTrue(self.armSubsystem.resetToZeroCommand())

  def _periodic(self) -> None:
    self._updateTelemetry()

  def getAutoCommand(self) -> Command:
    return self.autoCommands.getSelected()

  def autoInit(self) -> None:
    self.resetRobot()

  def autoExit(self) -> None: 
    self.gyroSensor.resetRobotToField(self.localizationService.getRobotPose())

  def teleopInit(self) -> None:
    self.resetRobot()

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or True

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())

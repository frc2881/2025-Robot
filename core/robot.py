from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.classes import RobotState, TargetAlignmentMode
from lib.controllers.game_controller import GameController
from lib.controllers.lights_controller import LightsController
from lib.sensors.gyro_sensor_navx2 import GyroSensor_NAVX2
from lib.sensors.pose_sensor import PoseSensor
from core.commands.auto import AutoCommands
from core.commands.game import GameCommands
from core.subsystems.drive import DriveSubsystem
from core.subsystems.elevator import ElevatorSubsystem
from core.subsystems.arm import ArmSubsystem
from core.subsystems.wrist import WristSubsystem
from core.subsystems.hand import HandSubsystem
from core.services.localization import LocalizationService
from core.classes import TargetAlignmentLocation, TargetPositionType, WristPosition, GamePiece, LightsMode
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
    self._setupDriverControls()
    self._setupOperatorControls()
    self._setupLightsControls()

  def _setupDriverControls(self) -> None:
    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveCommand(
        self.driverController.getLeftY,
        self.driverController.getLeftX,
        self.driverController.getRightX
    ))
    self.driverController.rightStick().and_((self.driverController.rightBumper().or_(self.driverController.leftBumper())).not_()).whileTrue(
      self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Center)
    )
    self.driverController.rightStick().and_(self.driverController.rightBumper()).whileTrue(
      self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Right)
    )
    self.driverController.rightStick().and_(self.driverController.leftBumper()).whileTrue(
      self.gameCommands.alignRobotToTargetCommand(TargetAlignmentMode.Translation, TargetAlignmentLocation.Left)
    )
    self.driverController.leftStick().whileTrue(
      self.driveSubsystem.lockCommand()
    )
    # self.driverController.rightTrigger().whileTrue(cmd.none())
    # self.driverController.leftTrigger().whileTrue(cmd.none())
    # self.driverController.rightBumper().whileTrue(cmd.none())
    # self.driverController.leftBumper().whileTrue(cmd.none())
    # self.driverController.povUp().and_((self.driverController.start()).not_()).whileTrue(cmd.none())
    # self.driverController.povDown().and_((self.driverController.start()).not_()).whileTrue(cmd.none())
    # self.driverController.povLeft().and_((self.driverController.start()).not_()).whileTrue(cmd.none())
    # self.driverController.povRight().and_((self.driverController.start()).not_()).whileTrue(cmd.none())
    # self.driverController.a().whileTrue(cmd.none())
    # self.driverController.b().whileTrue(cmd.none())
    # self.driverController.y().whileTrue(cmd.none())
    self.driverController.x().whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.CageClimb)
    )
    # self.driverController.start().and_((
    #     self.driverController.povLeft()
    #     .or_(self.driverController.povUp())
    #     .or_(self.driverController.povRight())
    #     .or_(self.driverController.povDown())
    #   ).not_()
    # ).onTrue(cmd.none())
    self.driverController.start().and_(self.driverController.povLeft()).whileTrue(
      self.autoCommands.moveToStartingPosition(1)
    )
    self.driverController.start().and_(self.driverController.povUp()).whileTrue(
      self.autoCommands.moveToStartingPosition(2)
    )
    self.driverController.start().and_(self.driverController.povRight()).whileTrue(
      self.autoCommands.moveToStartingPosition(3)
    )
    self.driverController.back().onTrue(
      self.gyroSensor.resetCommand()
    )

  def _setupOperatorControls(self) -> None:
    self.elevatorSubsystem.setDefaultCommand(
      self.elevatorSubsystem.runCommand(
        self.operatorController.getLeftY
    ))
    self.armSubsystem.setDefaultCommand(
      self.armSubsystem.runCommand(
        self.operatorController.getRightY
    ))
    self.operatorController.leftTrigger().onTrue(
      self.gameCommands.intakeCommand(GamePiece.Coral)
    )
    self.operatorController.rightTrigger().onTrue(
      self.gameCommands.scoreCommand(GamePiece.Coral)
    )
    self.operatorController.leftBumper().whileTrue(
      self.gameCommands.intakeCommand(GamePiece.Algae)
    )
    self.operatorController.rightBumper().onTrue(
      self.gameCommands.scoreCommand(GamePiece.Algae)
    )
    self.operatorController.povUp().and_((self.operatorController.start()).not_()).whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.ReefCoralL4)
    )
    self.operatorController.povRight().and_((self.operatorController.start()).not_()).whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.ReefCoralL3)
    )
    self.operatorController.povDown().and_((self.operatorController.start()).not_()).whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.ReefCoralL2)
    )
    self.operatorController.povLeft().and_((self.operatorController.start()).not_()).whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.ReefCoralL1)
    )
    self.operatorController.povUpRight().whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.ReefAlgaeL3)
    )
    self.operatorController.povDownRight().whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.ReefAlgaeL2)
    )
    self.operatorController.povDownLeft().whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.AlgaeProcessor)
    )
    self.operatorController.povUpLeft().whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.Barge)
    )
    self.operatorController.a().whileTrue(
      self.gameCommands.intakeCoralCommand()
    )
    # self.operatorController.b().whileTrue(cmd.none())
    self.operatorController.y().onTrue(
      self.wristSubsystem.togglePositionCommand()
    )
    self.operatorController.x().whileTrue(
      self.gameCommands.alignRobotToTargetPositionCommand(TargetPositionType.CageEntry)
    )
    # self.operatorController.start().and_((
    #     self.operatorController.povLeft()
    #     .or_(self.operatorController.povUp())
    #     .or_(self.operatorController.povRight())
    #     .or_(self.operatorController.povDown())
    #   ).not_()
    # ).whileTrue(cmd.none())
    self.operatorController.start().and_(self.operatorController.povDown()).whileTrue(
      self.elevatorSubsystem.resetToZeroLowerStageCommand()
    )
    self.operatorController.start().and_(self.operatorController.povUp()).whileTrue(
      self.elevatorSubsystem.resetToZeroUpperStageCommand()
    )
    self.operatorController.start().and_(self.operatorController.povRight()).whileTrue(
      self.armSubsystem.resetToZeroCommand()
    )
    self.operatorController.start().and_(self.operatorController.povLeft()).whileTrue(
      self.wristSubsystem.setPositionCommand(WristPosition.Up)
    )
    # self.operatorController.back().onTrue(cmd.none())

  def _setupLightsControls(self) -> None:
    self.lightsController = LightsController()
    utils.addRobotPeriodic(self._updateLights)

  def _updateLights(self) -> None:
    lightsMode = LightsMode.Default
    match utils.getRobotState():
      case RobotState.Disabled:
        if not self._robotHasInitialZeroResets():
          lightsMode = LightsMode.RobotNotReady
        elif not self.localizationService.hasVisionTarget():
          lightsMode = LightsMode.VisionNotReady
      case RobotState.Enabled:
        # TODO: add additional modes as needed for game/robot state visualization for drive team
        pass
    self.lightsController.setLightsMode(lightsMode.name)

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

  def disabledInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()
    self.elevatorSubsystem.reset()
    self.armSubsystem.reset()
    self.wristSubsystem.reset()
    self.handSubsystem.reset()

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or (
      self.elevatorSubsystem.hasInitialZeroReset() and self.armSubsystem.hasInitialZeroReset()
    )

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())

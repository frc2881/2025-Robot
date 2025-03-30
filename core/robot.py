from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.xbox import Xbox
from lib.sensors.distance import DistanceSensor
from lib.sensors.gyro_navx2 import Gyro_NAVX2
from lib.sensors.pose import PoseSensor
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive
from core.subsystems.elevator import Elevator
from core.subsystems.arm import Arm
from core.subsystems.wrist import Wrist
from core.subsystems.hand import Hand
from core.subsystems.shield import Shield
from core.subsystems.intake import Intake
from core.services.localization import Localization
from core.services.lights import Lights
from core.classes import TargetAlignmentLocation, TargetPositionType, ElevatorStage
import core.constants as constants

class RobotCore:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initServices()
    self._initControllers()
    self._initCommands()
    self._initTriggers()
    self._initLights()
    utils.addRobotPeriodic(self._periodic)

  def _initSensors(self) -> None:
    self.gyro = Gyro_NAVX2(constants.Sensors.Gyro.NAVX2.kComType)
    self.poseSensors = tuple(PoseSensor(c) for c in constants.Sensors.Pose.kPoseSensorConfigs)
    self.gripperDistanceSensor = DistanceSensor(constants.Sensors.Distance.Gripper.kConfig)
    self.intakeDistanceSensor = DistanceSensor(constants.Sensors.Distance.Intake.kConfig)
    SmartDashboard.putString("Robot/Sensors/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))

  def _initSubsystems(self) -> None:
    self.drive = Drive(self.gyro.getHeading)
    self.elevator = Elevator()
    self.arm = Arm()
    self.wrist = Wrist()
    self.hand = Hand(self.gripperDistanceSensor.hasTarget)
    self.intake = Intake(self.intakeDistanceSensor.hasTarget)
    self.shield = Shield()
    
  def _initServices(self) -> None:
    self.localization = Localization(
      self.gyro.getRotation, 
      self.drive.getModulePositions, 
      self.poseSensors, 
      self.drive.isAligningToTarget
    )

  def _initControllers(self) -> None:
    self.driver = Xbox(constants.Controllers.kDriverControllerPort, constants.Controllers.kInputDeadband)
    self.operator = Xbox(constants.Controllers.kOperatorControllerPort, constants.Controllers.kInputDeadband)
    DriverStation.silenceJoystickConnectionWarning(not utils.isCompetitionMode())

  def _initCommands(self) -> None:
    self.game = Game(self)
    self.auto = Auto(self)

  def _initTriggers(self) -> None:
    self._setupDriver()
    self._setupOperator()

  def _setupDriver(self) -> None:
    self.drive.setDefaultCommand(
      self.drive.default(self.driver.getLeftY, self.driver.getLeftX, self.driver.getRightX)
    )
    self.driver.rightStick().and_((self.driver.rightBumper().or_(self.driver.leftBumper())).not_()).whileTrue(
      self.game.alignRobotToTarget(TargetAlignmentMode.Translation, TargetAlignmentLocation.Center)
    )
    self.driver.rightStick().and_(self.driver.rightBumper()).whileTrue(
      self.game.alignRobotToTarget(TargetAlignmentMode.Translation, TargetAlignmentLocation.Right)
    )
    self.driver.rightStick().and_(self.driver.leftBumper()).whileTrue(
      self.game.alignRobotToTarget(TargetAlignmentMode.Translation, TargetAlignmentLocation.Left)
    )
    self.driver.leftStick().whileTrue(
      self.drive.lock()
    )
    self.driver.rightTrigger().whileTrue(
      self.game.intake()
    )
    # self.driver.leftTrigger().whileTrue(cmd.none())
    # self.driver.rightBumper().whileTrue(cmd.none())
    # self.driver.leftBumper().whileTrue(cmd.none())
    # self.driver.povUp().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    self.driver.povDown().and_((self.driver.start()).not_()).whileTrue(
      self.intake.eject()
    )
    # self.driver.povLeft().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.povRight().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.a().onTrue(cmd.none())
    # self.driver.b().whileTrue(cmd.none())
    self.driver.y().whileTrue(
      self.elevator.default(lambda: constants.Subsystems.Elevator.kCageDeepClimbDownSpeed, ElevatorStage.Lower)
    )
    self.driver.x().whileTrue(
      self.elevator.default(lambda: constants.Subsystems.Elevator.kCageDeepClimbUpSpeed, ElevatorStage.Lower)
    )
    self.driver.start().and_((
        self.driver.povLeft()
        .or_(self.driver.povUp())
        .or_(self.driver.povRight())
        .or_(self.driver.povDown())
      ).not_()
    ).whileTrue(
      self.intake.resetToZero()
    )

    self.driver.start().and_(self.driver.povLeft()).whileTrue(
      self.auto.moveToStartingPosition(1)
    )
    self.driver.start().and_(self.driver.povUp()).whileTrue(
      self.auto.moveToStartingPosition(2)
    )
    self.driver.start().and_(self.driver.povRight()).whileTrue(
      self.auto.moveToStartingPosition(3)
    )
    self.driver.back().onTrue(
      self.gyro.reset()
    )

  def _setupOperator(self) -> None:
    self.elevator.setDefaultCommand(
      self.elevator.default(self.operator.getLeftY)
    )
    self.intake.setDefaultCommand(
      self.intake.default()
    )
    self.arm.setDefaultCommand(
      self.arm.default(self.operator.getRightY) # TODO: .onlyIf(lambda: not self.intake.isIntakeUp())
    )
    self.operator.leftTrigger().whileTrue(
      self.game.intakeGripper()
    )
    self.operator.rightTrigger().onTrue(
      self.game.score()
    )
    self.operator.leftBumper().whileTrue(
      self.game.moveCoralToGripper()
    )
    # self.operator.rightBumper().whileTrue(
    #   self.intake.alignToPosition(constants.Subsystems.Intake.kHandoffPosition)
    # )
    self.operator.povUp().and_((self.operator.start()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL4)
    )
    self.operator.povRight().and_((self.operator.start()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL3)
    )
    self.operator.povDown().and_((self.operator.start()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL2)
    )
    self.operator.povLeft().and_((self.operator.start()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL1)
    )
    self.operator.a().whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.CoralStation)
    )
    self.operator.b().whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefAlgaeL2)
    )
    self.operator.y().whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefAlgaeL3)
    )
    self.operator.x().whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.CageDeepClimb)
    )
    self.operator.start().and_(self.operator.povDown()).whileTrue(
      self.elevator.resetLowerStageToZero()
    )
    self.operator.start().and_(self.operator.povUp()).whileTrue(
      self.elevator.resetUpperStageToZero()
    )
    self.operator.start().and_(self.operator.povRight()).whileTrue(
      self.wrist.togglePosition()
    )
    self.operator.start().and_(self.operator.povLeft()).whileTrue(
      self.arm.resetToZero()
    )
    self.operator.start().and_((
        self.operator.povLeft()
        .or_(self.operator.povUp())
        .or_(self.operator.povRight())
        .or_(self.operator.povDown())
      ).not_()
    ).whileTrue(
      self.elevator.default(self.operator.getLeftY, ElevatorStage.Upper)
    )
    self.operator.back().whileTrue(
      self.elevator.default(self.operator.getLeftY, ElevatorStage.Lower)
    )

  def _initLights(self) -> None:
    self.lights = Lights(
      self._hasAllZeroResets,
      self.localization.hasVisionTarget,
      self.game.isGripperHolding,
      self.game.isRobotAlignedForScoring
    )

  def _periodic(self) -> None:
    self._updateTelemetry()

  def disabledInit(self) -> None:
    self.reset()

  def autoInit(self) -> None:
    self.reset()

  def autoExit(self) -> None: 
    self.gyro.resetRobotToField(self.localization.getRobotPose())

  def teleopInit(self) -> None:
    self.reset()

  def testInit(self) -> None:
    self.reset()

  def simulationInit(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.drive.reset()
    self.elevator.reset()
    self.arm.reset()
    self.intake.reset()
    self.wrist.reset()
    self.hand.reset()
    self.shield.reset()

  def _hasAllZeroResets(self) -> bool:
    return utils.isCompetitionMode() or (
      self.elevator.hasZeroReset() and self.arm.hasZeroReset()
    )

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/HasAllZeroResets", self._hasAllZeroResets())

from commands2 import Command, cmd
from wpilib import DriverStation, SmartDashboard
from lib import logger, utils
from lib.classes import TargetAlignmentMode
from lib.controllers.xbox import Xbox
from lib.sensors.distance import DistanceSensor
from lib.sensors.beambreak import BeamBreakSensor
from lib.sensors.gyro_navx2 import Gyro_NAVX2
from lib.sensors.pose import PoseSensor
from core.commands.auto import Auto
from core.commands.game import Game
from core.subsystems.drive import Drive
from core.subsystems.elevator import Elevator
from core.subsystems.arm import Arm
from core.subsystems.wrist import Wrist
from core.subsystems.hand import Hand
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
    # self.gripperSensor = DistanceSensor(constants.Sensors.Distance.Gripper.kConfig
    self.gripperSensor = BeamBreakSensor("Gripper", constants.Sensors.BeamBreak.Gripper.kChannel) 
    self.intakeSensor = BeamBreakSensor("Intake", constants.Sensors.BeamBreak.Intake.kChannel) 
    SmartDashboard.putString("Robot/Sensors/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))

  def _initSubsystems(self) -> None:
    self.drive = Drive(self.gyro.getHeading)
    self.elevator = Elevator()
    self.arm = Arm()
    self.wrist = Wrist()
    self.hand = Hand(self.gripperSensor.hasTarget)
    self.intake = Intake(self.intakeSensor.hasTarget)
    
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
    self.driver.leftTrigger().whileTrue(
      self.game.intakeCoral()
    )
    self.driver.rightTrigger().whileTrue(
      self.game.scoreCoral()
    )
    # self.driver.rightBumper().whileTrue(cmd.none())
    # self.driver.leftBumper().whileTrue(cmd.none())
    # self.driver.povUp().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.povDown().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.povLeft().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.povRight().and_((self.driver.start()).not_()).whileTrue(cmd.none())
    # self.driver.a().whileTrue(cmd.none())
    self.driver.b().whileTrue(
      self.intake.eject()
    )
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
    self.arm.setDefaultCommand(
      self.arm.default(self.operator.getRightY)
    )
    self.operator.leftTrigger().whileTrue(
      self.game.runGripper()
    )
    self.operator.rightTrigger().whileTrue(
      self.game.scoreCoral()
    )
    self.operator.leftBumper().whileTrue(
      self.hand.holdGripper()
    )
    self.operator.rightBumper().whileTrue(
      self.game.liftCoral()
    )
    self.operator.povUp().and_((self.operator.start()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL4)
    )
    self.operator.povRight().and_((self.operator.start()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL3)
    )
    self.operator.povDown().and_((self.operator.start()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefCoralL2)
    )
    #self.operator.povLeft().whileTrue(cmd.none())
    self.operator.a().whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.CoralStation)
    )
    self.operator.b().whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefAlgaeL2)
    )
    self.operator.y().whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.ReefAlgaeL3)
    )
    self.operator.x().and_((self.operator.povLeft()).not_()).whileTrue(
      self.game.alignRobotToTargetPosition(TargetPositionType.CageIntercept)
    )
    self.operator.x().and_((self.operator.povLeft())).whileTrue(
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
      self.localization.hasValidVisionTarget,
      self.game.isRobotAlignedForScoring,
      self.game.isGripperHolding
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
    self.wrist.reset()
    self.hand.reset()
    self.intake.reset()

  def _hasAllZeroResets(self) -> bool:
    return (
      self.elevator.hasZeroReset() and self.arm.hasZeroReset() 
      if not utils.isCompetitionMode() else 
      True
    )
      
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Status/HasAllZeroResets", self._hasAllZeroResets())

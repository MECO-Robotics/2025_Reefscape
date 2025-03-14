package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralPreset;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ElevatorCommands.ELEVATOR_HEIGHT_PRESETS;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.components.Components;
import frc.robot.subsystems.drive.AutoDriveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOReplay;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSim;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOTalonFX;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOReplay;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSim;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOTalonFX;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOReplay;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIOReplay;
import frc.robot.subsystems.position_joint.PositionJointIOSim;
import frc.robot.subsystems.position_joint.PositionJointIOSparkMax;
import frc.robot.subsystems.position_joint.PositionJointIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionTrig;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import frc.robot.util.controller.ControllerUtil;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private PositionJoint rightCoralRotationMotor;
  private Flywheel rightCoralRollerMotor;
  private PositionJoint leftCoralRotationMotor;
  private Flywheel leftCoralRollerMotor;
  private PositionJoint elevatorMotor;
  private PositionJoint elbowMotor;
  // private Flywheel endEffectorMotor;

  // Just in case
  private final LoggedNetworkBoolean reefA;
  private final LoggedNetworkBoolean reefB;
  private final LoggedNetworkBoolean reefC;
  private final LoggedNetworkBoolean reefD;
  private final LoggedNetworkBoolean reefE;
  private final LoggedNetworkBoolean reefF;
  private final LoggedNetworkBoolean reefG;
  private final LoggedNetworkBoolean reefH;
  private final LoggedNetworkBoolean reefI;
  private final LoggedNetworkBoolean reefJ;
  private final LoggedNetworkBoolean reefK;
  private final LoggedNetworkBoolean reefL;

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController coPilotController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        // If using REV hardware, set up the Spark Odometry Thread, if using CTRE
        // hardware, set up
        // the Phoenix Odometry Thread, if using a combination of the two, set up both
        drive =
            new Drive(
                new GyroIOPigeon2(13, DriveMotorConstants.canBusName),
                new Module(
                    new DriveMotorIOTalonFX(
                        "FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOTalonFX(
                        "FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG)),
                new Module(
                    new DriveMotorIOTalonFX("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOTalonFX(
                        "BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    new AzimuthMotorIOTalonFX(
                        "BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG)),
                DriveMotorConstants.DRIVE_GAINS,
                AzimuthMotorConstants.AZMITH_GAINS,
                PhoenixOdometryThread.getInstance(),
                null);

        // Coral Intake rotation motor
        rightCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOSparkMax(
                    "RightCoralRotateMotor",
                    PositionJointConstants.RIGHT_CORAL_INTAKE_RROTATION_CONFIG),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
        rightCoralRollerMotor =
            new Flywheel(
                new FlywheelIOSparkMax(
                    "RightCoralRollerMotor", FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLERS_CONFG),
                FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

        leftCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOSparkMax(
                    "LeftCoralRotateMotor",
                    PositionJointConstants.LEFT_CORAL_INTAKE_RROTATION_CONFIG),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
        leftCoralRollerMotor =
            new Flywheel(
                new FlywheelIOSparkMax(
                    "LeftCoralRollerMotor", FlywheelConstants.LEFT_CORAL_INTAKE_ROLLERS_CONFG),
                FlywheelConstants.LEFT_CORAL_INTAKE_ROLLER_GAINS);

        // Elevator
        elevatorMotor =
            new PositionJoint(
                new PositionJointIOSparkMax(
                    "ElevatorMotor", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS);

        // Elbow
        elbowMotor =
            new PositionJoint(
                new PositionJointIOTalonFX("ElbowMotor", PositionJointConstants.ELBOW_CONFIG),
                PositionJointConstants.PIVOT_GAINS);

        // End Effector
        // endEffectorMotor = new Flywheel(
        // new FlywheelIOSparkMax("EndEffectorMotor",
        // FlywheelConstants.END_EFFECTOR_CONFIG),
        // FlywheelConstants.END_EFFECTOR_GAINS);

        VisionIOQuestNav questNav =
            new VisionIOQuestNav(
                VisionConstants.robotToQuest,
                new VisionIOPhotonVisionTrig(
                    VisionConstants.frontTagCamera,
                    VisionConstants.robotToFrontTagCamera,
                    drive::getRotation));

        // vision
        vision = new Vision(drive::addVisionMeasurement, questNav);

        driverController.b().onTrue(Commands.runOnce(questNav::resetPose).ignoringDisable(true));

        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOSim("FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    new AzimuthMotorIOSim("FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    new AzimuthMotorIOSim(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    new AzimuthMotorIOSim("BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG)),
                new Module(
                    new DriveMotorIOSim("BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    new AzimuthMotorIOSim("BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG)),
                DriveMotorConstants.DRIVE_GAINS_SIM,
                AzimuthMotorConstants.AZMITH_GAINS_SIM,
                null,
                null);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOQuestNav(
                    VisionConstants.robotToQuest,
                    new VisionIOPhotonVisionTrig(
                        VisionConstants.frontTagCamera,
                        VisionConstants.robotToFrontTagCamera,
                        drive::getRotation)));

        rightCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOSim(
                    "RightCoralRotateMotor",
                    PositionJointConstants.RIGHT_CORAL_INTAKE_RROTATION_CONFIG),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
        rightCoralRollerMotor =
            new Flywheel(
                new FlywheelIOSim(
                    "RightCoralRollerMotor", FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLERS_CONFG),
                FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

        leftCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOSim(
                    "LeftCoralRotateMotor",
                    PositionJointConstants.LEFT_CORAL_INTAKE_RROTATION_CONFIG),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
        leftCoralRollerMotor =
            new Flywheel(
                new FlywheelIOSim(
                    "LeftCoralRollerMotor", FlywheelConstants.LEFT_CORAL_INTAKE_ROLLERS_CONFG),
                FlywheelConstants.LEFT_CORAL_INTAKE_ROLLER_GAINS);

        elevatorMotor =
            new PositionJoint(
                new PositionJointIOSim("ElevatorMotor", PositionJointConstants.ELEVATOR_CONFIG),
                PositionJointConstants.ELEVATOR_GAINS_SIM);

        elbowMotor =
            new PositionJoint(
                new PositionJointIOSim("ElbowMotor", PositionJointConstants.ELBOW_CONFIG),
                PositionJointConstants.PIVOT_GAINS_SIM);

        // endEffectorMotor = new Flywheel(
        // new FlywheelIOSim("EndEffectorMotor", FlywheelConstants.END_EFFECTOR_CONFIG),
        // FlywheelConstants.END_EFFECTOR_GAINS);

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOReplay("FrontLeftDrive"),
                    new AzimuthMotorIOReplay("FrontLeftAz")),
                new Module(
                    new DriveMotorIOReplay("FrontRightDrive"),
                    new AzimuthMotorIOReplay("FrontRightAz")),
                new Module(
                    new DriveMotorIOReplay("BackLeftDrive"),
                    new AzimuthMotorIOReplay("BackLeftAz")),
                new Module(
                    new DriveMotorIOReplay("BackRightDrive"),
                    new AzimuthMotorIOReplay("BackRightAz")),
                DriveMotorConstants.DRIVE_GAINS,
                AzimuthMotorConstants.AZMITH_GAINS,
                null,
                null);
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        rightCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOReplay("RightCoralRotateMotor"),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);

        rightCoralRollerMotor =
            new Flywheel(
                new FlywheelIOReplay("RightCoralRollerMotor"),
                FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

        leftCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOReplay("LeftCoralRotateMotor"),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);

        leftCoralRollerMotor =
            new Flywheel(
                new FlywheelIOReplay("LeftCoralRollerMotor"),
                FlywheelConstants.LEFT_CORAL_INTAKE_ROLLER_GAINS);

        elevatorMotor =
            new PositionJoint(
                new PositionJointIOReplay("ElevatorMotor"), PositionJointConstants.ELEVATOR_GAINS);

        elbowMotor =
            new PositionJoint(
                new PositionJointIOReplay("ElbowMotor"), PositionJointConstants.PIVOT_GAINS);

        // endEffectorMotor = new Flywheel(
        // new FlywheelIOReplay("EndEffectorMotor"),
        // FlywheelConstants.END_EFFECTOR_GAINS);

        break;
    }
    /*
     * CommandScheduler.getInstance().schedule(Commands.sequence(
     * Commands.waitSeconds(1),
     * Commands.runOnce(questNav::reset
     * ));
     */

    // Set up auto routines

    NamedCommands.registerCommand(
        "LIntake", IntakeCommands.deployIntake(leftCoralRotationMotor, leftCoralRollerMotor));
    NamedCommands.registerCommand(
        "LStow", IntakeCommands.stowIntake(leftCoralRotationMotor, leftCoralRollerMotor));
    NamedCommands.registerCommand(
        "RIntake", IntakeCommands.deployIntake(rightCoralRotationMotor, rightCoralRollerMotor));
    NamedCommands.registerCommand(
        "RStow", IntakeCommands.stowIntake(rightCoralRotationMotor, rightCoralRollerMotor));

    NamedCommands.registerCommand(
        "One",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_ONE_CORAL));
    NamedCommands.registerCommand(
        "Two",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL));
    NamedCommands.registerCommand(
        "Three",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL));
    NamedCommands.registerCommand(
        "Four",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));
    NamedCommands.registerCommand(
        "CoralWait",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL));

    NamedCommands.registerCommand(
        "Handoff",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.HANDOFF));

    NamedCommands.registerCommand("Score", ElevatorCommands.scorePreset(elbowMotor));

    // Need to create the NamedCommands before the autoChooser or else they won't
    // work
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();

    // subsystems
    new Components(
        elevatorMotor::getPosition,
        elbowMotor::getPosition,
        rightCoralRotationMotor::getPosition,
        leftCoralRotationMotor::getPosition);

    reefA = new LoggedNetworkBoolean("/Presets/ReefA", false);
    reefB = new LoggedNetworkBoolean("/Presets/ReefB", false);
    reefC = new LoggedNetworkBoolean("/Presets/ReefC", false);
    reefD = new LoggedNetworkBoolean("/Presets/ReefD", false);
    reefE = new LoggedNetworkBoolean("/Presets/ReefE", false);
    reefF = new LoggedNetworkBoolean("/Presets/ReefF", false);
    reefG = new LoggedNetworkBoolean("/Presets/ReefG", false);
    reefH = new LoggedNetworkBoolean("/Presets/ReefH", false);
    reefI = new LoggedNetworkBoolean("/Presets/ReefI", false);
    reefJ = new LoggedNetworkBoolean("/Presets/ReefJ", false);
    reefK = new LoggedNetworkBoolean("/Presets/ReefK", false);
    reefL = new LoggedNetworkBoolean("/Presets/ReefL", false);
  }

  Pose2d reefAPos = new Pose2d(3.17, 4.18, Rotation2d.fromDegrees(0));
  Pose2d reefBPos = new Pose2d(3.17, 3.865, Rotation2d.fromDegrees(0));
  Pose2d reefCPos = new Pose2d(3.7, 3.0, Rotation2d.fromDegrees(61.2));
  Pose2d reefDPos = new Pose2d(3.98, 2.79, Rotation2d.fromDegrees(61.2));
  Pose2d reefEPos = new Pose2d(5.01, 2.81, Rotation2d.fromDegrees(122.5));
  Pose2d reefFPos = new Pose2d(5.232, 3.061, Rotation2d.fromDegrees(122.5));
  Pose2d reefGPos = new Pose2d(5.82, 3.86, Rotation2d.fromDegrees(180));
  Pose2d reefHPos = new Pose2d(5.82, 4.19, Rotation2d.fromDegrees(180));
  Pose2d reefIPos = new Pose2d(5.33, 5.06, Rotation2d.fromDegrees(-118.3));
  Pose2d reefJPos = new Pose2d(5.01, 5.25, Rotation2d.fromDegrees(-118.3));
  Pose2d reefKPos = new Pose2d(3.98, 5.23, Rotation2d.fromDegrees(-58.7));
  Pose2d reefLPos = new Pose2d(3.68, 5.05, Rotation2d.fromDegrees(-58.7));

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro / odometry
    final Runnable resetGyro =
        () ->
            drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    DriverStation.getAlliance().isPresent()
                        ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                            ? new Rotation2d(Math.PI)
                            : new Rotation2d())
                        : new Rotation2d())); // zero gyro

    // Reset gyro to 0° when B button is pressed
    driverController.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    // Riases the elevator and wrist to the prests for L4
    // Also incorpeates the safety algorithm for the wirst and elevator
    driverController
        .povUp()
        .onTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));

    coPilotController
        .povUp()
        .onTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));

    driverController.povDown().onTrue(ElevatorCommands.handOff(elevatorMotor, elbowMotor));

    coPilotController.povDown().onTrue(ElevatorCommands.handOff(elevatorMotor, elbowMotor));

    driverController
        .y()
        .onTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor,
                elbowMotor,
                ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL));

    // Coral Intake
    driverController // Right bumper to deploy right coral intake
        .rightBumper()
        .whileTrue(IntakeCommands.deployIntake(rightCoralRotationMotor, rightCoralRollerMotor))
        .whileFalse(IntakeCommands.stowIntake(rightCoralRotationMotor, rightCoralRollerMotor));

    driverController // Left bumper to deploy left coral intake
        .leftBumper()
        .whileTrue(IntakeCommands.deployIntake(leftCoralRotationMotor, leftCoralRollerMotor))
        .whileFalse(IntakeCommands.stowIntake(leftCoralRotationMotor, leftCoralRollerMotor));

    // Trigger for the elevator positions
    // Shows up on the dashboard
    // Right side reef

    // Co-pilot controller elevator values
    coPilotController
        .x()
        .whileTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_ONE_CORAL));
    coPilotController
        .a()
        .whileTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL));
    coPilotController
        .b()
        .whileTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL));
    coPilotController
        .y()
        .whileTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));

    // Co-pilot controller stow and handoff values
    coPilotController
        .povDown()
        .whileTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.HANDOFF));
    coPilotController
        .povUp()
        .whileTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor,
                elbowMotor,
                ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL));

    coPilotController.rightBumper().onTrue(autoDrive(0));
    coPilotController.leftBumper().onTrue(autoDrive(1));
    coPilotController
        .a()
        .onTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor, elbowMotor, ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));
    coPilotController.b().onTrue(ElevatorCommands.handOff(elevatorMotor, elbowMotor));

    new Trigger(reefA::get).onTrue(DriveCommands.pathfindToPose(drive, reefAPos));
    new Trigger(reefB::get).onTrue(DriveCommands.pathfindToPose(drive, reefBPos));
    new Trigger(reefC::get).onTrue(DriveCommands.pathfindToPose(drive, reefCPos));
    new Trigger(reefD::get).onTrue(DriveCommands.pathfindToPose(drive, reefDPos));
    new Trigger(reefE::get).onTrue(DriveCommands.pathfindToPose(drive, reefEPos));
    new Trigger(reefF::get).onTrue(DriveCommands.pathfindToPose(drive, reefFPos));
    new Trigger(reefG::get).onTrue(DriveCommands.pathfindToPose(drive, reefGPos));
    new Trigger(reefH::get).onTrue(DriveCommands.pathfindToPose(drive, reefHPos));
    new Trigger(reefI::get).onTrue(DriveCommands.pathfindToPose(drive, reefIPos));
    new Trigger(reefI::get).onTrue(DriveCommands.pathfindToPose(drive, reefIPos));
    new Trigger(reefJ::get).onTrue(DriveCommands.pathfindToPose(drive, reefJPos));
    new Trigger(reefK::get).onTrue(DriveCommands.pathfindToPose(drive, reefKPos));
    new Trigger(reefL::get).onTrue(DriveCommands.pathfindToPose(drive, reefLPos));

    // Trigger for the reef positions that goes on the touchscreen
    new Trigger(reefA::get)
        .or(reefB::get)
        .or(reefC::get)
        .or(reefD::get)
        .or(reefE::get)
        .or(reefF::get)
        .or(reefG::get)
        .or(reefH::get)
        .or(reefI::get)
        .or(reefJ::get)
        .or(reefK::get)
        .or(reefL::get)
        .onTrue(
            new InstantCommand(
                () -> {
                  reefA.set(false);
                  reefB.set(false);
                  reefC.set(false);
                  reefD.set(false);
                  reefE.set(false);
                  reefF.set(false);
                  reefG.set(false);
                  reefH.set(false);
                  reefI.set(false);
                  reefJ.set(false);
                  reefK.set(false);
                  reefL.set(false);
                }));
  }

  // -----------------------------------------------------------

  private Command autoDrive(int index) {
    Supplier<Pose2d> selectedReefPose =
        () ->
            AutoDriveConstants.reefPoses[index][
                ControllerUtil.getIndex(
                    coPilotController.getLeftX(), coPilotController.getLeftY())];

    Supplier<CoralPreset> selectedPreset =
        () ->
            AutoDriveConstants.coralPresets[
                ControllerUtil.getIndex(
                    coPilotController.getRightX(), coPilotController.getRightY())];

    return Commands.defer(
            () -> DriveCommands.pathfindToPose(drive, selectedReefPose.get()), Set.of(drive))
        .asProxy()
        .alongWith(
            Commands.defer(
                () ->
                    new WaitUntilCommand(
                            () ->
                                selectedReefPose
                                        .get()
                                        .getTranslation()
                                        .getDistance(drive.getPose().getTranslation())
                                    < AutoDriveConstants.DISTANCE_THRESH)
                        .andThen(
                            ElevatorCommands.moveSafe(
                                elevatorMotor, elbowMotor, selectedPreset.get())),
                Set.of(elevatorMotor, elbowMotor)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

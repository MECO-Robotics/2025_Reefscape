package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
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
import frc.robot.subsystems.piece_detection.PieceDetection;
import frc.robot.subsystems.piece_detection.PieceDetectionConstants;
import frc.robot.subsystems.piece_detection.PieceDetectionIOPhoton;
import frc.robot.subsystems.piece_detection.PieceDetectionIOReplay;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIOReplay;
import frc.robot.subsystems.position_joint.PositionJointIOSim;
import frc.robot.subsystems.position_joint.PositionJointIOSparkMax;
import frc.robot.subsystems.position_joint.PositionJointIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionGoon;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.controller.ControllerUtil;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

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

  // private PositionJoint climber;
  // private Flywheel climberRoller;

  private Flywheel endEffectorMotor;

  @SuppressWarnings("unused")
  private final Vision vision;

  @SuppressWarnings("unused")
  private final PieceDetection leftPieceDetection;

  @SuppressWarnings("unused")
  private final PieceDetection rightPieceDetection;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController coPilotController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final LoggedNetworkString selectedReef = new LoggedNetworkString("Selected Reef", "None");
  private final LoggedNetworkString selectedSide = new LoggedNetworkString("Selected Side", "None");
  private final LoggedNetworkString selectedLevel =
      new LoggedNetworkString("Selected Level", "None");

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

        // climber =
        // new PositionJoint(
        // new PositionJointIOSparkMax("ClimberMotor",
        // PositionJointConstants.CLIMBER_CONFIG),
        // PositionJointConstants.CLIMBER_GAINS);

        // climberRoller =
        // new Flywheel(
        // new FlywheelIOSparkMax("ClimberRoller",
        // FlywheelConstants.CLIMBER_ROLLER_CONFIG),
        // FlywheelConstants.CLIMBER_ROLLER_GAINS);

        // End Effector
        endEffectorMotor =
            new Flywheel(
                new FlywheelIOSparkMax("EndEffectorMotor", FlywheelConstants.END_EFFECTOR_CONFIG),
                FlywheelConstants.END_EFFECTOR_GAINS);

        // vision
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionGoon(
                    "LeftCamera", VisionConstants.robotToLeftTagCamera, drive::getRotation),
                new VisionIOPhotonVisionGoon(
                    "RightCamera", VisionConstants.robotToRightTagCamera, drive::getRotation));

        leftPieceDetection =
            new PieceDetection(
                new PieceDetectionIOPhoton(
                    "LeftPieceDetection", PieceDetectionConstants.LEFT_CONFIG, 0),
                () -> new Pose3d(drive.getPose()));
        rightPieceDetection =
            new PieceDetection(
                new PieceDetectionIOPhoton(
                    "RightPieceDetection", PieceDetectionConstants.RIGHT_CONFIG, 0),
                () -> new Pose3d(drive.getPose()));

        // driverController.b().onTrue(Commands.runOnce(questNav::resetHeading).ignoringDisable(true));
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
                new VisionIOPhotonVisionSim(
                    "LeftCamera", VisionConstants.robotToLeftTagCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    "RightCamera", VisionConstants.robotToRightTagCamera, drive::getPose));

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

        endEffectorMotor =
            new Flywheel(
                new FlywheelIOSim("EndEffectorMotor", FlywheelConstants.END_EFFECTOR_CONFIG),
                FlywheelConstants.END_EFFECTOR_GAINS);

        leftPieceDetection =
            new PieceDetection(
                new PieceDetectionIOReplay("LeftPieceDetection"),
                () -> new Pose3d(drive.getPose()));
        rightPieceDetection =
            new PieceDetection(
                new PieceDetectionIOReplay("RightPieceDetection"),
                () -> new Pose3d(drive.getPose()));

        // climber =
        // new PositionJoint(
        // new PositionJointIOSim("ClimberMotor",
        // PositionJointConstants.CLIMBER_CONFIG),
        // PositionJointConstants.CLIMBER_GAINS);

        // climberRoller =
        // new Flywheel(
        // new FlywheelIOSim("ClimberRoller", FlywheelConstants.CLIMBER_ROLLER_CONFIG),
        // FlywheelConstants.CLIMBER_ROLLER_GAINS);
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

        endEffectorMotor =
            new Flywheel(
                new FlywheelIOReplay("EndEffectorMotor"), FlywheelConstants.END_EFFECTOR_GAINS);

        leftPieceDetection =
            new PieceDetection(
                new PieceDetectionIOReplay("LeftPieceDetection"),
                () -> new Pose3d(drive.getPose()));
        rightPieceDetection =
            new PieceDetection(
                new PieceDetectionIOReplay("RightPieceDetection"),
                () -> new Pose3d(drive.getPose()));

        // climber =
        // new PositionJoint(
        // new PositionJointIOReplay("ClimberMotor"),
        // PositionJointConstants.CLIMBER_GAINS);

        // climberRoller =
        // new Flywheel(
        // new FlywheelIOReplay("ClimberRoller"),
        // FlywheelConstants.CLIMBER_ROLLER_GAINS);
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
        ElevatorCommands.moveSequential(
            elevatorMotor, elbowMotor, () -> ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_ONE_CORAL));
    NamedCommands.registerCommand(
        "Two",
        ElevatorCommands.moveSequential(
            elevatorMotor, elbowMotor, () -> ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL));
    NamedCommands.registerCommand(
        "Three",
        ElevatorCommands.moveSequential(
            elevatorMotor,
            elbowMotor,
            () -> ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL));
    NamedCommands.registerCommand(
        "Four",
        ElevatorCommands.moveSequential(
            elevatorMotor,
            elbowMotor,
            () -> ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));
    NamedCommands.registerCommand(
        "CoralWait",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL));

    NamedCommands.registerCommand(
        "Handoff",
        ElevatorCommands.moveSafe(
            elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.HANDOFF));

    NamedCommands.registerCommand("Score", ElevatorCommands.scorePreset(elbowMotor));

    NamedCommands.registerCommand("AlignToE", DriveCommands.autoAlignAuto(vision, "Left", "EF"));

    NamedCommands.registerCommand("AlignToF", DriveCommands.autoAlignAuto(vision, "Right", "EF"));

    NamedCommands.registerCommand("AlignToC", DriveCommands.autoAlignAuto(vision, "Left", "CD"));

    NamedCommands.registerCommand("AlignToD", DriveCommands.autoAlignAuto(vision, "Right", "CD"));

    NamedCommands.registerCommand(
        "ClearOverrides",
        Commands.runOnce(
            () -> AdvancedPPHolonomicDriveController.clearYFeedbackOverrideRobotRelative()));
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

    driverController
        .rightTrigger(.1)
        .onTrue(
            IntakeCommands.emergencyOuttake(rightCoralRollerMotor)
                .alongWith(IntakeCommands.emergencyOuttake(leftCoralRollerMotor)));
    driverController
        .rightTrigger(0.1)
        .onTrue(
            IntakeCommands.stop(rightCoralRollerMotor)
                .alongWith(IntakeCommands.stop(leftCoralRollerMotor)));

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

    // driverController
    // .y()
    // .onTrue(
    // ElevatorCommands.moveSafe(
    // elevatorMotor,
    // elbowMotor,
    // ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL));

    // Coral Intake
    driverController // Right bumper to deploy right coral intake
        .rightBumper()
        .whileTrue(
            IntakeCommands.deployIntake(rightCoralRotationMotor, rightCoralRollerMotor)
                .alongWith(
                    new FlywheelVoltageCommand(
                        leftCoralRollerMotor, IntakeCommands.ROLLER_VOLTS.INTAKE)))
        .whileFalse(
            IntakeCommands.stowIntake(rightCoralRotationMotor, rightCoralRollerMotor)
                .alongWith(
                    new FlywheelVoltageCommand(
                        leftCoralRollerMotor, IntakeCommands.ROLLER_VOLTS.STOP)));

    driverController // Left bumper to deploy left coral intake
        .leftBumper()
        .whileTrue(
            IntakeCommands.deployIntake(leftCoralRotationMotor, leftCoralRollerMotor)
                .alongWith(
                    new FlywheelVoltageCommand(
                        rightCoralRollerMotor, IntakeCommands.ROLLER_VOLTS.INTAKE)))
        .whileFalse(
            IntakeCommands.stowIntake(leftCoralRotationMotor, leftCoralRollerMotor)
                .alongWith(
                    new FlywheelVoltageCommand(
                        rightCoralRollerMotor, IntakeCommands.ROLLER_VOLTS.STOP)));

    driverController
        .y()
        .whileTrue(
            DriveCommands.joystickDriveToReef(
                drive,
                vision,
                elevatorMotor,
                elbowMotor,
                leftCoralRotationMotor,
                rightCoralRotationMotor,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                selectedReef::get,
                selectedSide::get,
                selectedLevel::get))
        .onFalse(
            IntakeCommands.stowIntake(leftCoralRotationMotor, leftCoralRollerMotor)
                .alongWith(
                    IntakeCommands.stowIntake(rightCoralRotationMotor, rightCoralRollerMotor)));

    driverController
        .povUp()
        .whileTrue(
            DriveCommands.joystickDriveRobotRelative(drive, () -> 0.5, () -> 0.0, () -> 0.0));
    driverController
        .povLeft()
        .whileTrue(
            DriveCommands.joystickDriveRobotRelative(drive, () -> 0.5, () -> 0.25, () -> 0.0));
    driverController
        .povRight()
        .whileTrue(
            DriveCommands.joystickDriveRobotRelative(drive, () -> 0.5, () -> -0.25, () -> 0.0));

    driverController
        .axisGreaterThan(2, 0.3)
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -0.5 * driverController.getLeftY(),
                () -> -0.5 * driverController.getLeftX(),
                () -> -0.5 * driverController.getRightX()));

    // Trigger for the elevator positions
    // Shows up on the dashboard
    // Right side reef

    // Co-pilot controller elevator values
    // coPilotController
    // .x()

    // .whileTrue(
    // ElevatorCommands.moveSafe(
    // elevatorMotor, elbowMotor,
    // ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_ONE_CORAL));
    // coPilotController
    // .a()
    // .whileTrue(
    // ElevatorCommands.moveSafe(
    // elevatorMotor, elbowMotor,
    // ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL));
    // coPilotController
    // .b()
    // .whileTrue(
    // ElevatorCommands.moveSafe(
    // elevatorMotor, elbowMotor,
    // ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL));
    // coPilotController
    // .y()
    // .whileTrue(
    // ElevatorCommands.moveSafe(
    // elevatorMotor, elbowMotor,
    // ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));

    // Co-pilot controller stow and handoff values
    coPilotController.povDown().whileTrue(ElevatorCommands.handOff(elevatorMotor, elbowMotor));
    coPilotController
        .povUp()
        .whileTrue(
            ElevatorCommands.moveSafe(
                elevatorMotor,
                elbowMotor,
                ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL));

    coPilotController.povRight().onTrue(ElevatorCommands.scorePreset(elbowMotor));

    // coPilotController.rightBumper().onTrue(autoDrive(0));
    // coPilotController.leftBumper().onTrue(autoDrive(1));

    coPilotController.x().whileTrue(ElevatorCommands.scorePreset(elbowMotor));

    // coPilotController
    // .rightBumper()
    // .onTrue(PositionJoint.setVoltage(climber, () -> 6.0))
    // .onFalse(PositionJoint.setVoltage(climber, () -> 0.0));

    // coPilotController
    // .leftBumper()
    // .onTrue(PositionJoint.setVoltage(climber, () -> -6.0))
    // .onFalse(PositionJoint.setVoltage(climber, () -> 0.0));

    // climberRoller.setDefaultCommand(
    // Flywheel.setVoltage(
    // climberRoller,
    // () ->
    // coPilotController.getLeftTriggerAxis() -
    // coPilotController.getRightTriggerAxis()));

    new Trigger(
            () ->
                ControllerUtil.getIndex(coPilotController.getLeftX(), coPilotController.getLeftY())
                    == 4)
        .onTrue(new InstantCommand(() -> selectedReef.set("AB")));

    new Trigger(
            () ->
                ControllerUtil.getIndex(coPilotController.getLeftX(), coPilotController.getLeftY())
                    == 3)
        .onTrue(new InstantCommand(() -> selectedReef.set("CD")));

    new Trigger(
            () ->
                ControllerUtil.getIndex(coPilotController.getLeftX(), coPilotController.getLeftY())
                    == 2)
        .onTrue(new InstantCommand(() -> selectedReef.set("EF")));

    new Trigger(
            () ->
                ControllerUtil.getIndex(coPilotController.getLeftX(), coPilotController.getLeftY())
                    == 1)
        .onTrue(new InstantCommand(() -> selectedReef.set("GH")));

    new Trigger(
            () ->
                ControllerUtil.getIndex(coPilotController.getLeftX(), coPilotController.getLeftY())
                    == 0)
        .onTrue(new InstantCommand(() -> selectedReef.set("IJ")));

    new Trigger(
            () ->
                ControllerUtil.getIndex(coPilotController.getLeftX(), coPilotController.getLeftY())
                    == 5)
        .onTrue(new InstantCommand(() -> selectedReef.set("KL")));

    // TODO: Change for actual buttons
    coPilotController.leftBumper().onTrue(new InstantCommand(() -> selectedSide.set("Left")));
    coPilotController.rightBumper().onTrue(new InstantCommand(() -> selectedSide.set("Right")));

    coPilotController.x().onTrue(new InstantCommand(() -> selectedLevel.set("L1")));
    coPilotController.a().onTrue(new InstantCommand(() -> selectedLevel.set("L2")));
    coPilotController.b().onTrue(new InstantCommand(() -> selectedLevel.set("L3")));
    coPilotController.y().onTrue(new InstantCommand(() -> selectedLevel.set("L4")));

    coPilotController
        .povLeft()
        .onTrue(
            ElevatorCommands.moveSequential(
                elevatorMotor,
                elbowMotor,
                () -> {
                  switch (selectedLevel.get()) {
                    case "L1":
                      return ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_ONE_CORAL;
                    case "L2":
                      return ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL;
                    case "L3":
                      return ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL;
                    case "L4":
                      return ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL;
                    default:
                      return ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL;
                  }
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

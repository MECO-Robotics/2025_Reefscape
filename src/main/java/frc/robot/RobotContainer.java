package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.components.Components;
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
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuestNavRelative;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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
    private Flywheel endEffectorMotor;
    private VisionIOQuestNavRelative questNav;

    @SuppressWarnings("unused")
    private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController coPilotController = new CommandXboxController(1);

    private final LoggedNetworkBoolean l1;

    private final LoggedNetworkBoolean l2;;

    private final LoggedNetworkBoolean l3;

    private final LoggedNetworkBoolean l4;

    private final LoggedNetworkBoolean HANDOFF;

    private final LoggedNetworkBoolean Score;

    private final LoggedNetworkBoolean Reattempt;

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

    private final LoggedNetworkBoolean resetPoseRed;
    private final LoggedNetworkBoolean resetPoseBlue;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations

                // If using REV hardware, set up the Spark Odometry Thread, if using CTRE
                // hardware, set up
                // the Phoenix Odometry Thread, if using a combination of the two, set up both
                drive = new Drive(
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
                rightCoralRotationMotor = new PositionJoint(
                        new PositionJointIOSparkMax(
                                "RightCoralRotateMotor",
                                PositionJointConstants.RIGHT_CORAL_INTAKE_RROTATION_CONFIG),
                        PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
                rightCoralRollerMotor = new Flywheel(
                        new FlywheelIOSparkMax(
                                "RightCoralRollerMotor", FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLERS_CONFG),
                        FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

                leftCoralRotationMotor = new PositionJoint(
                        new PositionJointIOSparkMax(
                                "LeftCoralRotateMotor",
                                PositionJointConstants.LEFT_CORAL_INTAKE_RROTATION_CONFIG),
                        PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
                leftCoralRollerMotor = new Flywheel(
                        new FlywheelIOSparkMax(
                                "LeftCoralRollerMotor", FlywheelConstants.LEFT_CORAL_INTAKE_ROLLERS_CONFG),
                        FlywheelConstants.LEFT_CORAL_INTAKE_ROLLER_GAINS);

                // Elevator
                elevatorMotor = new PositionJoint(
                        new PositionJointIOSparkMax(
                                "ElevatorMotor", PositionJointConstants.ELEVATOR_CONFIG),
                        PositionJointConstants.ELEVATOR_GAINS);

                // Elbow
                elbowMotor = new PositionJoint(
                        new PositionJointIOTalonFX("ElbowMotor", PositionJointConstants.ELBOW_CONFIG),
                        PositionJointConstants.PIVOT_GAINS);

                // End Effector
                endEffectorMotor = new Flywheel(
                        new FlywheelIOSparkMax("EndEffectorMotor", FlywheelConstants.END_EFFECTOR_CONFIG),
                        FlywheelConstants.END_EFFECTOR_GAINS);

                // questNav =
                // new VisionIOQuestNav(
                // VisionConstants.robotToCamera0,
                // new VisionIOPhotonVisionTrig(
                // "USB_Camera", VisionConstants.robotToCamera1, drive::getRotation));

                questNav = new VisionIOQuestNavRelative(VisionConstants.robotToCamera0);

                // vision
                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOQuestNavRelative(VisionConstants.robotToCamera0));

                break;

            case SIM:
                drive = new Drive(
                        new GyroIO() {
                        },
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

                vision = new Vision(
                        drive::addVisionMeasurement,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                        new VisionIOPhotonVisionSim(
                                VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

                rightCoralRotationMotor = new PositionJoint(
                        new PositionJointIOSim(
                                "RightCoralRotateMotor",
                                PositionJointConstants.RIGHT_CORAL_INTAKE_RROTATION_CONFIG),
                        PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
                rightCoralRollerMotor = new Flywheel(
                        new FlywheelIOSim(
                                "RightCoralRollerMotor", FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLERS_CONFG),
                        FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

                leftCoralRotationMotor = new PositionJoint(
                        new PositionJointIOSim(
                                "LeftCoralRotateMotor",
                                PositionJointConstants.LEFT_CORAL_INTAKE_RROTATION_CONFIG),
                        PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
                leftCoralRollerMotor = new Flywheel(
                        new FlywheelIOSim(
                                "LeftCoralRollerMotor", FlywheelConstants.LEFT_CORAL_INTAKE_ROLLERS_CONFG),
                        FlywheelConstants.LEFT_CORAL_INTAKE_ROLLER_GAINS);

                elevatorMotor = new PositionJoint(
                        new PositionJointIOSim("ElevatorMotor", PositionJointConstants.ELEVATOR_CONFIG),
                        PositionJointConstants.ELEVATOR_GAINS_SIM);

                elbowMotor = new PositionJoint(
                        new PositionJointIOSim("ElbowMotor", PositionJointConstants.ELBOW_CONFIG),
                        PositionJointConstants.PIVOT_GAINS_SIM);

                endEffectorMotor = new Flywheel(
                        new FlywheelIOSim("EndEffectorMotor", FlywheelConstants.END_EFFECTOR_CONFIG),
                        FlywheelConstants.END_EFFECTOR_GAINS);

                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
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
                vision = new Vision(drive::addVisionMeasurement, new VisionIO() {
                }, new VisionIO() {
                });

                rightCoralRotationMotor = new PositionJoint(
                        new PositionJointIOReplay("RightCoralRotateMotor"),
                        PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);

                rightCoralRollerMotor = new Flywheel(
                        new FlywheelIOReplay("RightCoralRollerMotor"),
                        FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

                leftCoralRotationMotor = new PositionJoint(
                        new PositionJointIOReplay("LeftCoralRotateMotor"),
                        PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);

                leftCoralRollerMotor = new Flywheel(
                        new FlywheelIOReplay("LeftCoralRollerMotor"),
                        FlywheelConstants.LEFT_CORAL_INTAKE_ROLLER_GAINS);

                elevatorMotor = new PositionJoint(
                        new PositionJointIOReplay("ElevatorMotor"), PositionJointConstants.ELEVATOR_GAINS);

                elbowMotor = new PositionJoint(
                        new PositionJointIOReplay("ElbowMotor"), PositionJointConstants.PIVOT_GAINS);

                endEffectorMotor = new Flywheel(
                        new FlywheelIOReplay("EndEffectorMotor"), FlywheelConstants.END_EFFECTOR_GAINS);

                break;
        }
        /*
         * CommandScheduler.getInstance().schedule(Commands.sequence(
         * Commands.waitSeconds(1),
         * Commands.runOnce(questNav::reset
         * ));
         */

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        l1 = new LoggedNetworkBoolean("/Presets/L1", false);
        l2 = new LoggedNetworkBoolean("/Presets/L2", false);
        l3 = new LoggedNetworkBoolean("/Presets/L3", false);
        l4 = new LoggedNetworkBoolean("/Presets/L4", false);
        HANDOFF = new LoggedNetworkBoolean("/Presets/HANDOFF", false);
        Score = new LoggedNetworkBoolean("/Score/", false);
        Reattempt = new LoggedNetworkBoolean("/Reattempt/", false);

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

        resetPoseRed = new LoggedNetworkBoolean("QuestReset/ResetRed", false);
        resetPoseBlue = new LoggedNetworkBoolean("QuestReset/ResetBlue", false);

        NamedCommands.registerCommand(
                "Intake", IntakeCommands.deployIntake(rightCoralRotationMotor, rightCoralRollerMotor));
        NamedCommands.registerCommand(
                "Stow Intake", IntakeCommands.stowIntake(rightCoralRotationMotor, rightCoralRollerMotor));

        NamedCommands.registerCommand(
                "Elevator L1",
                ElevatorCommands.setPreset(
                        elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_ONE_CORAL));
        NamedCommands.registerCommand(
                "Elevator L2",
                ElevatorCommands.setPreset(
                        elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL));
        NamedCommands.registerCommand(
                "Elevator L3",
                ElevatorCommands.setPreset(
                        elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL));
        NamedCommands.registerCommand(
                "Elevator L4",
                ElevatorCommands.setPreset(
                        elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));
        NamedCommands.registerCommand(
                "Stow",
                ElevatorCommands.setPreset(
                        elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.STOW));

        NamedCommands.registerCommand("Score", ElevatorCommands.scorePreset(elevatorMotor, elbowMotor));

        // Configure the button bindings
        configureButtonBindings();

        // TODO: Remove "filler" and replace with actual getPosition methods of actual
        // subsystems
        DoubleSupplier filler = () -> 0;
        new Components(
                elevatorMotor::getPosition,
                filler,
                rightCoralRotationMotor::getPosition,
                leftCoralRotationMotor::getPosition);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
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

        Pose2d reefAPos = new Pose2d(3.347, 4.181, Rotation2d.fromDegrees(0));
        Pose2d reefBPos = new Pose2d(3.304, 3.869, Rotation2d.fromDegrees(0));
        Pose2d reefCPos = new Pose2d(3.730, 3.104, Rotation2d.fromDegrees(55.4));
        Pose2d reefDPos = new Pose2d(4.056, 2.948, Rotation2d.fromDegrees(54.7));
        Pose2d reefEPos = new Pose2d(4.948, 2.877, Rotation2d.fromDegrees(119.1));
        Pose2d reefFPos = new Pose2d(5.232, 3.061, Rotation2d.fromDegrees(122.3));
        Pose2d reefGPos = new Pose2d(5.685, 3.869, Rotation2d.fromDegrees(-178.6));
        Pose2d reefHPos = new Pose2d(5.671, 4.181, Rotation2d.fromDegrees(-178.6));
        Pose2d reefIPos = new Pose2d(5.232, 4.932, Rotation2d.fromDegrees(-117.8));
        Pose2d reefJPos = new Pose2d(4.92, 5.102, Rotation2d.fromDegrees(-117.8));
        Pose2d reefKPos = new Pose2d(4.041, 5.130, Rotation2d.fromDegrees(-61.3));
        Pose2d reefLPos = new Pose2d(3.772, 4.954, Rotation2d.fromDegrees(-61.9));
        // // Reset gyro / odometry
        final Runnable resetGyro = () -> drive.setPose(
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
                        ElevatorCommands.setPreset(
                                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));

    coPilotController
        .povUp()
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));

    driverController
        .povDown()
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.HANDOFF));

    coPilotController
        .povDown()
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.HANDOFF));

    driverController
        .y()
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor,
                elbowMotor,
                ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL));

    coPilotController
        .y()
        .onTrue(
            ElevatorCommands.setPreset(
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

    coPilotController.x().whileTrue(ElevatorCommands.scorePreset(elevatorMotor, elbowMotor));

    // Trigger for the elevator positions
    // Shows up on the dashboard
    new Trigger(l1::get)
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.HANDOFF));
    new Trigger(l2::get)
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL));
    new Trigger(l3::get)
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL));
    new Trigger(l4::get)
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL));
    new Trigger(HANDOFF::get)
        .onTrue(
            ElevatorCommands.setPreset(
                elevatorMotor, elbowMotor, ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.HANDOFF));
    new Trigger(Score::get).onTrue(ElevatorCommands.scorePreset(elevatorMotor, elbowMotor));
    new Trigger(Reattempt::get).onTrue(ElevatorCommands.scoreReattempt(elevatorMotor, elbowMotor));

        new Trigger(l1::get)
                .or(new Trigger(l2::get))
                .or(new Trigger(l3::get))
                .or(new Trigger(l4::get))
                .or(new Trigger(HANDOFF::get))
                .or(new Trigger(Score::get))
                .or(new Trigger(Reattempt::get))
                .onTrue(
                        new InstantCommand(
                                () -> {
                                    l1.set(false);
                                    l2.set(false);
                                    l3.set(false);
                                    l4.set(false);
                                    HANDOFF.set(false);
                                    Score.set(false);
                                    Reattempt.set(false);

                                    if (elevatorMotor.getCurrentCommand() != null) {
                                        elevatorMotor.getCurrentCommand().cancel();
                                    }

                                    if (elbowMotor.getCurrentCommand() != null) {
                                        elbowMotor.getCurrentCommand().cancel();
                                    }
                                }));

        // Auto Driving
        // Poses for each of the different branches in the reef
        // See 4481's strategy sheet to find out which branches are which letter

        new Trigger(reefA::get).onTrue(DriveCommands.pathfindToPose(drive, reefAPos));
        new Trigger(reefB::get).onTrue(DriveCommands.pathfindToPose(drive, reefBPos));
        new Trigger(reefC::get).onTrue(DriveCommands.pathfindToPose(drive, reefCPos));
        new Trigger(reefD::get).onTrue(DriveCommands.pathfindToPose(drive, reefDPos));
        new Trigger(reefE::get).onTrue(DriveCommands.pathfindToPose(drive, reefEPos));
        new Trigger(reefF::get).onTrue(DriveCommands.pathfindToPose(drive, reefFPos));
        new Trigger(reefG::get).onTrue(DriveCommands.pathfindToPose(drive, reefGPos));
        new Trigger(reefH::get).onTrue(DriveCommands.pathfindToPose(drive, reefHPos));
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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
  private Flywheel endEffectorMotor;

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

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
        endEffectorMotor =
            new Flywheel(
                new FlywheelIOSparkMax("EndEffectorMotor", FlywheelConstants.END_EFFECTOR_CONFIG),
                FlywheelConstants.END_EFFECTOR_GAINS);

        // Vision
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0));
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
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));

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
                PositionJointConstants.ELEVATOR_GAINS);

        elbowMotor =
            new PositionJoint(
                new PositionJointIOSim("ElbowMotor", PositionJointConstants.ELBOW_CONFIG),
                PositionJointConstants.PIVOT_GAINS);

        endEffectorMotor =
            new Flywheel(
                new FlywheelIOSim("EndEffectorMotor", FlywheelConstants.END_EFFECTOR_CONFIG),
                FlywheelConstants.END_EFFECTOR_GAINS);

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

        break;
    }

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

    // Coral Intake
    driverController // Right bumper to deploy right coral intake
        .rightBumper()
        .whileTrue(IntakeCommands.deployIntake(rightCoralRotationMotor, rightCoralRollerMotor))
        .whileFalse(IntakeCommands.stowIntake(rightCoralRotationMotor, rightCoralRollerMotor));

    driverController // Left bumper to deploy left coral intake
        .leftBumper()
        .whileTrue(IntakeCommands.deployIntake(leftCoralRotationMotor, leftCoralRollerMotor))
        .whileFalse(IntakeCommands.stowIntake(leftCoralRotationMotor, leftCoralRollerMotor));

    driverController.povUp().whileTrue(ElevatorCommands.MAX(elevatorMotor));

    driverController.povDown().whileTrue(ElevatorCommands.handOff(elevatorMotor));
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

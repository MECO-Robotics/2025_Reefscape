package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import frc.robot.util.pathplanner.AllianceUtil;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

/** A collection of commands for controlling the drive subsystem. */
public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final LoggedTunableNumber ANGLE_KP = new LoggedTunableNumber("AngleKp", 5.0);
  private static final LoggedTunableNumber ANGLE_KD = new LoggedTunableNumber("AngleKd", 0.4);
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  public static final LoggedTunableNumber REEF_P = new LoggedTunableNumber("ReefP", 3.0);
  public static final LoggedTunableNumber REEF_MAX_VELO =
      new LoggedTunableNumber("ReefMaxVelo", 2.0);
  public static final LoggedTunableNumber REEF_MAX_ACCEL =
      new LoggedTunableNumber("ReefMaxAccel", 1.0);

  public static final LoggedTunableNumber APPROACH_SPEED =
      new LoggedTunableNumber("Approach Speed", 1.0);

  private DriveCommands() {}

  /**
   * Converts joystick input into linear velocity.
   *
   * @param x The x-axis value of the joystick.
   * @param y The y-axis value of the joystick.
   * @return The linear velocity.
   */
  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param drive The drive subsystem.
   * @param xSupplier The supplier for the x-axis value of the joystick.
   * @param ySupplier The supplier for the y-axis value of the joystick.
   * @param omegaSupplier The supplier for the angular velocity.
   * @return The command.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          speeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation());
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param drive The drive subsystem.
   * @param xSupplier The supplier for the x-axis value of the joystick.
   * @param ySupplier The supplier for the y-axis value of the joystick.
   * @param omegaSupplier The supplier for the angular velocity.
   * @return The command.
   */
  public static Command joystickDriveRobotRelative(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(speeds);
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   *
   * @param drive The drive subsystem.
   * @param xSupplier The supplier for the x-axis value of the joystick.
   * @param ySupplier The supplier for the y-axis value of the joystick.
   * @param rotationSupplier The supplier for the target rotation.
   * @return The command.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP.get(),
            0.0,
            ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());
              drive.runVelocity(speeds);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  public static final Command joystickDriveToReef(
      Drive drive,
      Vision vision,
      PositionJoint elevatorMotor,
      PositionJoint elbowMotor,
      PositionJoint rightCoralRotationMotor,
      PositionJoint leftCoralRotationMotor,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<String> reefSupplier,
      Supplier<String> sideSupplier,
      Supplier<String> levelSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP.get(),
            0.0,
            ANGLE_KD.get(),
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    ProfiledPIDController yController =
        new ProfiledPIDController(
            REEF_P.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(REEF_MAX_VELO.get(), REEF_MAX_ACCEL.get()));

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega;
              int cameraNum;

              Logger.recordOutput("Servo/SideSupplier", sideSupplier.get());

              switch (sideSupplier.get()) {
                case "Left":
                  cameraNum = 1;
                  break;
                case "Right":
                  cameraNum = 0;
                  break;
                default:
                  cameraNum = 0;
                  break;
              }

              Logger.recordOutput("Servo/SelectedCamera", cameraNum);
              Logger.recordOutput(
                  "Servo/DesiredTag", AllianceUtil.getTagIDFromReefAlliance(reefSupplier.get()));
              Logger.recordOutput(
                  "Servo/SelectedTargetData", vision.getLatestTargetObservation()[cameraNum]);

              // Face towards the desired tag
              omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(),
                      AllianceUtil.flipRotation2dAlliance(
                              AllianceUtil.getRotationFromReefTagID(
                                  AllianceUtil.getTagIDFromReefAlliance(reefSupplier.get())))
                          .getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              speeds =
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation());

              if (vision.hasTarget()[cameraNum]
                  && vision.getLatestTargetObservation()[cameraNum].tagId()
                      == AllianceUtil.getTagIDFromReefAlliance(reefSupplier.get())) {

                double effort =
                    yController.calculate(
                        vision.getLatestTargetObservation()[cameraNum].tx().getRadians(), 0.0);

                Logger.recordOutput("Servo/TargetEffort", effort);

                speeds.vyMetersPerSecond = effort;

                if (vision.getLatestTargetObservation()[cameraNum].ty().getDegrees() > 10) {
                  speeds.vxMetersPerSecond = APPROACH_SPEED.get();
                } else if (vision.getLatestTargetObservation()[cameraNum].ty().getDegrees() > -13) {
                  speeds.vxMetersPerSecond = 0.5 * APPROACH_SPEED.get();
                } else {
                  speeds.vxMetersPerSecond = 0;
                }

                // if (angleController.atGoal()
                // && Math.abs(vision.getLatestTargetObservation()[cameraNum].tx().getRadians())
                // < 0.05) {
                // speeds.vxMetersPerSecond = APPROACH_SPEED.get();
                // } else {
                // speeds.vxMetersPerSecond = 0;
                // }
              }

              drive.runVelocity(speeds);
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()))
        .beforeStarting(
            () -> {
              int cameraNum;
              switch (sideSupplier.get()) {
                case "Left":
                  cameraNum = 1;
                  break;
                case "Right":
                  cameraNum = 0;
                  break;
                default:
                  cameraNum = 1;
                  break;
              }
              yController.reset(vision.getLatestTargetObservation()[cameraNum].tx().getRadians());
            })
        .alongWith(
            Commands.waitUntil(
                    () -> {
                      int cameraNum;
                      switch (sideSupplier.get()) {
                        case "Left":
                          cameraNum = 1;
                          break;
                        case "Right":
                          cameraNum = 0;
                          break;
                        default:
                          cameraNum = 1;
                          break;
                      }

                      return vision.hasTarget()[cameraNum]
                          && vision.getLatestTargetObservation()[cameraNum].tagId()
                              == AllianceUtil.getTagIDFromReefAlliance(reefSupplier.get());
                    })
                .andThen(
                    ElevatorCommands.moveSafe(
                        elevatorMotor,
                        elbowMotor,
                        () -> {
                          switch (levelSupplier.get()) {
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
                        })))
        .alongWith(
            Commands.waitUntil(
                    () -> {
                      int cameraNum;
                      switch (sideSupplier.get()) {
                        case "Left":
                          cameraNum = 1;
                          break;
                        case "Right":
                          cameraNum = 0;
                          break;
                        default:
                          cameraNum = 1;
                          break;
                      }

                      return vision.hasTarget()[cameraNum]
                          && vision.getLatestTargetObservation()[cameraNum].tagId()
                              == AllianceUtil.getTagIDFromReefAlliance(reefSupplier.get());
                    })
                .andThen(
                    IntakeCommands.deployIntakeAlign(rightCoralRotationMotor)
                        .alongWith(IntakeCommands.deployIntakeAlign(leftCoralRotationMotor))));
  }

  public static Command autoAlignAuto(Vision vision, String reefSide, String reefFace) {
    int cameraNum;
    switch (reefSide) {
      case "Left":
        cameraNum = 1;
        break;
      case "Right":
        cameraNum = 0;
        break;
      default:
        cameraNum = 1;
        break;
    }

    ProfiledPIDController yController =
        new ProfiledPIDController(
            REEF_P.get(),
            0.0,
            0.0,
            new TrapezoidProfile.Constraints(REEF_MAX_VELO.get(), REEF_MAX_ACCEL.get()));

    return Commands.runOnce(
            () -> {
              AdvancedPPHolonomicDriveController.overrideYFeedbackRobotRelative(
                  () -> {
                    System.out.println("Overriding Y feedback");
                    Logger.recordOutput(
                        "Servo/DesiredTag",
                        vision.getLatestTargetObservation()[cameraNum].tagId()
                            == AllianceUtil.getTagIDFromReefAlliance(reefFace));

                    if (vision.hasTarget()[cameraNum]
                        && vision.getLatestTargetObservation()[cameraNum].tagId()
                            == AllianceUtil.getTagIDFromReefAlliance(reefFace)) {

                      double effort =
                          yController.calculate(
                              vision.getLatestTargetObservation()[cameraNum].tx().getRadians(),
                              0.0);

                      Logger.recordOutput("Servo/TargetEffort", effort);
                      return effort;
                    } else {
                      return 0;
                    }
                  });
            })
        .beforeStarting(
            () ->
                yController.reset(
                    vision.getLatestTargetObservation()[cameraNum].tx().getRadians()));
  }

  public static final Command pathfindToPose(Drive drive, Pose2d targetPose) {

    PathConstraints constraints =
        new PathConstraints(4, 4, Math.toRadians(360), Math.toRadians(360), 12, false);
    return AutoBuilder.pathfindToPose(targetPose, constraints, 1.0);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   *
   * @param drive The drive subsystem.
   * @return The command.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  public static Command pathfindToPath(Drive drive, String path) {
    try {
      return AutoBuilder.pathfindThenFollowPath(
          PathPlannerPath.fromPathFile(path),
          new PathConstraints(4, 4, Math.toRadians(360), Math.toRadians(360), 12, false));
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
    return Commands.none();
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * DriveConstants.driveBaseRadius) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  // public static Command azimuthTuning()

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}

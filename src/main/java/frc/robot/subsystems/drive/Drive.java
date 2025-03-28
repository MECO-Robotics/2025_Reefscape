package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants.DriveMotorGains;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.drive.odometry_threads.SparkOdometryThread;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import frc.robot.util.mechanical_advantage.swerve.ModuleLimits;
import frc.robot.util.mechanical_advantage.swerve.SwerveSetpoint;
import frc.robot.util.mechanical_advantage.swerve.SwerveSetpointGenerator;
import frc.robot.util.pathplanner.AdvancedPPHolonomicDriveController;
import frc.robot.util.pathplanner.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Drive", "Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final SwerveSetpointGenerator setpointGenerator;
  private ModuleLimits currentModuleLimits = DriveConstants.currentModuleLimits;
  private SwerveSetpoint currentSetpoint =
      new SwerveSetpoint(
          new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          },
          new double[4]);

  private final LoggedTunableNumber drivekP;
  private final LoggedTunableNumber drivekI;
  private final LoggedTunableNumber drivekD;
  private final LoggedTunableNumber drivekS;
  private final LoggedTunableNumber drivekV;
  private final LoggedTunableNumber drivekA;

  private final LoggedTunableNumber azimuthkP;
  private final LoggedTunableNumber azimuthkI;
  private final LoggedTunableNumber azimuthkD;
  private final LoggedTunableNumber azimuthkS;
  private final LoggedTunableNumber azimuthkV;
  private final LoggedTunableNumber azimuthkA;

  private final LoggedTunableNumber kMaxDriveVelocity;
  private final LoggedTunableNumber kMaxDriveAcceleration;
  private final LoggedTunableNumber kMaxDriveDeceleration;
  private final LoggedTunableNumber kMaxSteeringVelocity;

  public Drive(
      GyroIO gyroIO,
      Module flModule,
      Module frModuleIO,
      Module blModuleIO,
      Module brModuleIO,
      DriveMotorGains driveGains,
      AzimuthMotorGains azimuthGains,
      PhoenixOdometryThread phoenixOdometryThread,
      SparkOdometryThread sparkOdometryThread) {
    this.gyroIO = gyroIO;
    modules[0] = flModule;
    modules[1] = frModuleIO;
    modules[2] = blModuleIO;
    modules[3] = brModuleIO;

    setpointGenerator =
        new SwerveSetpointGenerator(
            kinematics,
            DriveConstants.moduleTranslations[0],
            DriveConstants.moduleTranslations[1],
            DriveConstants.moduleTranslations[2],
            DriveConstants.moduleTranslations[3]);

    drivekP = new LoggedTunableNumber("Drive/DriveMotors/Gains/kP", driveGains.kP());
    drivekI = new LoggedTunableNumber("Drive/DriveMotors/Gains/kI", driveGains.kI());
    drivekD = new LoggedTunableNumber("Drive/DriveMotors/Gains/kD", driveGains.kD());
    drivekS = new LoggedTunableNumber("Drive/DriveMotors/Gains/kS", driveGains.kS());
    drivekV = new LoggedTunableNumber("Drive/DriveMotors/Gains/kV", driveGains.kV());
    drivekA = new LoggedTunableNumber("Drive/DriveMotors/Gains/kA", driveGains.kA());

    azimuthkP = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kP", azimuthGains.kP());
    azimuthkI = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kI", azimuthGains.kI());
    azimuthkD = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kD", azimuthGains.kD());
    azimuthkS = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kS", azimuthGains.kS());
    azimuthkV = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kV", azimuthGains.kV());
    azimuthkA = new LoggedTunableNumber("Drive/AzimuthMotors/Gains/kA", azimuthGains.kA());

    kMaxDriveVelocity =
        new LoggedTunableNumber(
            "Drive/ModuleLimits/kMaxDriveVelocityMetersPerSec",
            currentModuleLimits.maxDriveVelocity());
    kMaxDriveAcceleration =
        new LoggedTunableNumber(
            "Drive/ModuleLimits/kMaxDriveAccelerationMetersPerSecSq",
            currentModuleLimits.maxDriveAcceleration());
    kMaxDriveDeceleration =
        new LoggedTunableNumber(
            "Drive/ModuleLimits/kMaxDriveDecelerationMetersPerSecSq",
            currentModuleLimits.maxDriveDeceleration());
    kMaxSteeringVelocity =
        new LoggedTunableNumber(
            "Drive/ModuleLimits/kMaxSteeringVelocityRadPerSec",
            currentModuleLimits.maxSteeringVelocity());

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start phoenix odometry thread
    if (phoenixOdometryThread != null) {
      phoenixOdometryThread.start();
    }

    // Start spark odometry thread
    if (sparkOdometryThread != null) {
      sparkOdometryThread.start();
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new AdvancedPPHolonomicDriveController(
            DriveConstants.translationPID, DriveConstants.rotationPID),
        DriveConstants.ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Drive/Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    flModule.setGains(driveGains, azimuthGains);
    frModuleIO.setGains(driveGains, azimuthGains);
    blModuleIO.setGains(driveGains, azimuthGains);
    brModuleIO.setGains(driveGains, azimuthGains);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Drive/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        try {
          rawGyroRotation = gyroInputs.odometryYawPositions[i];
        } catch (Exception e) {

        }
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          for (int i = 0; i < 4; i++) {
            modules[i].setGains(
                new DriveMotorGains(
                    values[0], values[1], values[2], values[3], values[4], values[5]),
                new AzimuthMotorGains(
                    values[6], values[7], values[8], values[9], values[10], values[11]));
          }
        },
        drivekP,
        drivekI,
        drivekD,
        drivekS,
        drivekV,
        drivekA,
        azimuthkP,
        azimuthkI,
        azimuthkD,
        azimuthkS,
        azimuthkV,
        azimuthkA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        (values) -> {
          currentModuleLimits = new ModuleLimits(values[0], values[1], values[2], values[3]);
        },
        kMaxDriveVelocity,
        kMaxDriveAcceleration,
        kMaxDriveDeceleration,
        kMaxSteeringVelocity);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    currentSetpoint =
        setpointGenerator.generateSetpoint(
            currentModuleLimits, currentSetpoint, speeds, new Translation2d(), 0.02);
    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/Setpoints", currentSetpoint.moduleStates());
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", speeds);

    Logger.recordOutput(
        "Drive/SwerveStates/AzimuthVelocityFF", currentSetpoint.azimuthVelocityFF());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(
          currentSetpoint.moduleStates()[i], currentSetpoint.azimuthVelocityFF()[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("Drive/SwerveStates/SetpointsOptimized", currentSetpoint.moduleStates());
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "Drive/SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Drive/Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // poseEstimator.addVisionMeasurement(
    // visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.maxSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DriveConstants.driveBaseRadius;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return DriveConstants.moduleTranslations;
  }
}

package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class PositionJointConstants {
  public enum GravityType {
    CONSTANT,
    COSINE,
    // Not supported by TalonFX
    SINE
  }

  public enum EncoderType {
    INTERNAL,
    EXTERNAL_CANCODER,
    EXTERNAL_CANCODER_PRO,
    EXTERNAL_DIO,
    EXTERNAL_SPARK
  }

  public record PositionJointGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kG,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel,
      double kMinPosition,
      double kMaxPosition,
      double kTolerance,
      double kDefaultSetpoint) {
  }

  // Position Joint Gear Ratio should be multiplied by Math.PI * 2 for rotation
  // joints to convert
  // from rotations to radians
  public record PositionJointHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      GravityType gravity,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {
  }

  public static final PositionJointGains EXAMPLE_GAINS = new PositionJointGains(0, 0, 0, 0.25, 0.85, 0, 0, 0, 0, -6, 0,
      0, 0);

  public static final PositionJointHardwareConfig EXAMPLE_CONFIG = new PositionJointHardwareConfig(
      new int[] { 10 },
      new boolean[] { true },
      85.33333 * 2 * Math.PI,
      40,
      GravityType.COSINE,
      EncoderType.EXTERNAL_CANCODER,
      11,
      Rotation2d.fromRotations(0.5),
      "");

  // Coral Intake
  public static final PositionJointGains CORAL_INTAKE_ROTATION_GAINS = new PositionJointGains(20, 0.0, 0.0, 0.5, 1.0,
      2.0, 0.0, 0.5, 0.5, 0.0, Math.PI, 0.2, 0.0);

  public static final PositionJointHardwareConfig RIGHT_CORAL_INTAKE_RROTATION_CONFIG = new PositionJointHardwareConfig(
      new int[] { 21 },
      new boolean[] { false },
      (3 * 5) * 24 / 16,
      40,
      GravityType.SINE,
      EncoderType.INTERNAL,
      -1,
      Rotation2d.fromRotations(0),
      "");

  public static final PositionJointHardwareConfig LEFT_CORAL_INTAKE_RROTATION_CONFIG = new PositionJointHardwareConfig(
      new int[] { 23 },
      new boolean[] { true },
      (3 * 5) * 24 / 16,
      40,
      GravityType.SINE,
      EncoderType.INTERNAL,
      -1,
      Rotation2d.fromRotations(0),
      "");

  public static final class CORAL_ROTATION_POSITIONS {
    private final LoggedTunableNumber UP = new LoggedTunableNumber("CoralIntakeRotation", 0);
    private final LoggedTunableNumber DOWN = new LoggedTunableNumber("CoralIntakePosition", 0);
    public static final LoggedTunableNumber UP = 0; // -0.28;
    public static final LoggedTunableNumber DOWN = -0.25; // -5.85;
  }

  // Elevator

}

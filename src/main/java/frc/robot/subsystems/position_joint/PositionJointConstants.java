package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
      double kDefaultSetpoint) {}

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
      String canBus) {}

  // Coral Intake
  public static final PositionJointGains CORAL_INTAKE_ROTATION_GAINS =
      new PositionJointGains(
          25, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 1.0, 4.0, 0.0, Units.degreesToRotations(101), 0.2, 0.0);

  public static final PositionJointHardwareConfig RIGHT_CORAL_INTAKE_RROTATION_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {20},
          new boolean[] {true},
          (3 * 5) * 24 / 16,
          40,
          GravityType.SINE,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0),
          "");

  public static final PositionJointHardwareConfig LEFT_CORAL_INTAKE_RROTATION_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {23},
          new boolean[] {false},
          (3 * 5) * 24 / 16,
          40,
          GravityType.SINE,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0),
          "");

  public static final PositionJointGains ELEVATOR_GAINS =
      new PositionJointGains(8, 0, 0, 0.26, 0.475, 3.5, .3, 5, 5.0, -0.287, 1.014, 0.01, 0);

  public static final PositionJointGains ELEVATOR_GAINS_SIM =
      new PositionJointGains(10, 0, 0, 0.25, 0.5, 0.75, 0, 8.0, 4.0, -0.287, 1.014, 0.01, 0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {30, 31},
          new boolean[] {false, true},
          (31.4 + 8.8) / Units.inchesToMeters(51),
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0),
          "");
  // 31.4 max
  // -8.88 stow
  public static final PositionJointGains PIVOT_GAINS =
      new PositionJointGains(80, 0, 0, 0, 0.27, 0, 0, 2, 2, -0.25, 0.35, 0.01, -0.25);

  public static final PositionJointGains PIVOT_GAINS_SIM =
      new PositionJointGains(80, 0, 0, 0, 0, 0, 0, 2, 2, -0.25, 0.53, 0.1, -0.25);

  public static final PositionJointHardwareConfig ELBOW_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {33},
          new boolean[] {true},
          (72.0 / 16.0) * (70.0 / 16.0),
          40,
          GravityType.COSINE,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(-0.25),
          "");

  public static final PositionJointGains CLIMBER_GAINS =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointGains CLIMBER_GAINS_SIM =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointHardwareConfig CLIMBER_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {40},
          new boolean[] {true},
          (72.0 / 16.0) * (70.0 / 16.0),
          40,
          GravityType.COSINE,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(-0.25),
          "");
}

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
          20, 0.0, 0.0, 0.5, 1.0, 2.0, 0.0, 1.0, 4.0, 0.0, Units.degreesToRotations(101), 0.2, 0.0);

  public static final PositionJointHardwareConfig RIGHT_CORAL_INTAKE_RROTATION_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {21},
          new boolean[] {false},
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
          new boolean[] {true},
          (3 * 5) * 24 / 16,
          40,
          GravityType.SINE,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0),
          "");

  public static final PositionJointGains ELEVATOR_GAINS =
      new PositionJointGains(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  public static final PositionJointHardwareConfig ELEVATOR_CONFIG =
      new PositionJointHardwareConfig(
          new int[] {30, 31},
          new boolean[] {false, true},
          1,
          40,
          GravityType.CONSTANT,
          EncoderType.INTERNAL,
          -1,
          Rotation2d.fromRotations(0),
          "");
}

package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.position_joint.PositionJointConstants.EncoderType;

public class AzimuthMotorConstants {
  public static final String canBusName = "MECO CANIvore";

  public record AzimuthMotorGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel) {}

  public record AzimuthMotorHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final AzimuthMotorHardwareConfig FRONT_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {1},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          12,
          Rotation2d.fromRotations(-0.448486328125),
          canBusName);

  public static final AzimuthMotorHardwareConfig FRONT_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {3},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          11,
          Rotation2d.fromRotations(-0.1064453125),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {7},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          9,
          Rotation2d.fromRotations(0.381591796875),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {5},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER,
          10,
          Rotation2d.fromRotations(0.30908203125),
          canBusName);

  public static final AzimuthMotorGains FRONT_LEFT_GAINS =
      new AzimuthMotorGains(25, 0, 0, 0, 2.5, 0, 5, 5);

  public static final AzimuthMotorGains FRONT_RIGHT_GAINS =
      new AzimuthMotorGains(25, 0, 0, 0, 2.5, 0, 5, 5);

  public static final AzimuthMotorGains BACK_LEFT_GAINS =
      new AzimuthMotorGains(25, 0, 0, 0, 2.5, 0, 5, 5);

  public static final AzimuthMotorGains BACK_RIGHT_GAINS =
      new AzimuthMotorGains(25, 0, 0, 0, 2.5, 0, 5, 5);
}

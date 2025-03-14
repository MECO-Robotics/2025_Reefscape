package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.position_joint.PositionJointConstants.EncoderType;

public class AzimuthMotorConstants {
  public static final String canBusName = "MECO CANIvore";

  public record AzimuthMotorGains(
      double kP, double kI, double kD, double kS, double kV, double kA) {}

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
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          12,
          Rotation2d.fromRotations(-0.448975),
          canBusName);

  public static final AzimuthMotorHardwareConfig FRONT_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {3},
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          11,
          Rotation2d.fromRotations(-0.091553),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {7},
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          9,
          Rotation2d.fromRotations(0.389648),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {5},
          new boolean[] {false},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_CANCODER_PRO,
          10,
          Rotation2d.fromRotations(0.312744),
          canBusName);

  public static final AzimuthMotorGains AZMITH_GAINS = new AzimuthMotorGains(15, 0, 0, 0.25, 2, 0);

  public static final AzimuthMotorGains AZMITH_GAINS_SIM =
      new AzimuthMotorGains(35, 0, 0, 0.0, 3, 0);
}

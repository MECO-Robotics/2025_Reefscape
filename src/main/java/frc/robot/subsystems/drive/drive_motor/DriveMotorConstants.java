package frc.robot.subsystems.drive.drive_motor;

import frc.robot.subsystems.drive.DriveConstants;

public class DriveMotorConstants {
  public static final String canBusName = "Drive";

  public record DriveMotorGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kMaxAccel) {}

  public record DriveMotorHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final DriveMotorHardwareConfig FRONT_LEFT_CONFIG =
      new DriveMotorHardwareConfig(
          new int[] {2}, new boolean[] {false}, DriveConstants.driveMotorGearRatio, canBusName);

  public static final DriveMotorHardwareConfig FRONT_RIGHT_CONFIG =
      new DriveMotorHardwareConfig(
          new int[] {4}, new boolean[] {false}, DriveConstants.driveMotorGearRatio, canBusName);

  public static final DriveMotorHardwareConfig BACK_LEFT_CONFIG =
      new DriveMotorHardwareConfig(
          new int[] {8}, new boolean[] {false}, DriveConstants.driveMotorGearRatio, canBusName);

  public static final DriveMotorHardwareConfig BACK_RIGHT_CONFIG =
      new DriveMotorHardwareConfig(
          new int[] {6}, new boolean[] {false}, DriveConstants.driveMotorGearRatio, canBusName);

  public static final DriveMotorGains FRONT_LEFT_GAINS =
      new DriveMotorGains(0.2, 0, 0, 0, 0.9, 0, 100);

  public static final DriveMotorGains FRONT_RIGHT_GAINS =
      new DriveMotorGains(0.2, 0, 0, 0, 0.9, 0, 100);

  public static final DriveMotorGains BACK_LEFT_GAINS =
      new DriveMotorGains(0.2, 0, 0, 0, 0.9, 0, 100);

  public static final DriveMotorGains BACK_RIGHT_GAINS =
      new DriveMotorGains(0.2, 0, 0, 0, 0.9, 0, 100);
}

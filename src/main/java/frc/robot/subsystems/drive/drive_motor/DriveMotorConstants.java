package frc.robot.subsystems.drive.drive_motor;

import frc.robot.subsystems.drive.DriveConstants;

public class DriveMotorConstants {
    public static final String canBusName = "MECO CANIvore";

    public record DriveMotorGains(double kP, double kI, double kD, double kS, double kV, double kA) {
    }

    public record DriveMotorHardwareConfig(
            int[] canIds, boolean[] reversed, double gearRatio, String canBus) {
    }

    public static final DriveMotorHardwareConfig FRONT_LEFT_CONFIG = new DriveMotorHardwareConfig(
            new int[] { 2 }, new boolean[] { true }, DriveConstants.driveMotorGearRatio, canBusName);

    public static final DriveMotorHardwareConfig FRONT_RIGHT_CONFIG = new DriveMotorHardwareConfig(
            new int[] { 4 }, new boolean[] { true }, DriveConstants.driveMotorGearRatio, canBusName);

    public static final DriveMotorHardwareConfig BACK_LEFT_CONFIG = new DriveMotorHardwareConfig(
            new int[] { 8 }, new boolean[] { true }, DriveConstants.driveMotorGearRatio, canBusName);

    public static final DriveMotorHardwareConfig BACK_RIGHT_CONFIG = new DriveMotorHardwareConfig(
            new int[] { 6 }, new boolean[] { true }, DriveConstants.driveMotorGearRatio, canBusName);

    public static final DriveMotorGains DRIVE_GAINS = new DriveMotorGains(3, 0, 0, 0.5, 1, 0);

    public static final DriveMotorGains DRIVE_GAINS_SIM = new DriveMotorGains(0.1, 0, 0, 0, 0.46, 0);
}

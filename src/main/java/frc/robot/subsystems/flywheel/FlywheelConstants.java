package frc.robot.subsystems.flywheel;

public class FlywheelConstants {
  public record FlywheelGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxAccel,
      double kTolerance) {}

  public record FlywheelHardwareConfig(
      int[] canIds, boolean[] reversed, double gearRatio, String canBus) {}

  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {true}, 2.0, "");

  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  // Coral Intake
  public static final FlywheelHardwareConfig RIGHT_CORAL_INTAKE_ROLLERS_CONFG =
      new FlywheelHardwareConfig(new int[] {0}, new boolean[] {false}, 2.0, "");
  public static final FlywheelGains RIGHT_CORAL_INTAKE_ROLLER_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  public static final FlywheelHardwareConfig LEFT_CORAL_INTAKE_ROLLERS_CONFG =
      new FlywheelHardwareConfig(new int[] {0}, new boolean[] {true}, 2.0, "");
  public static final FlywheelGains LEFT_CORAL_INTAKE_ROLLER_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);
}

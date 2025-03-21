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
      int[] canIds, boolean[] reversed, double gearRatio, int currentLimit, String canBus) {}

  public static final FlywheelHardwareConfig EXAMPLE_CONFIG =
      new FlywheelHardwareConfig(new int[] {1}, new boolean[] {true}, 2.0, 40, "");

  public static final FlywheelGains EXAMPLE_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  // Coral Intake
  public static final FlywheelHardwareConfig RIGHT_CORAL_INTAKE_ROLLERS_CONFG =
      new FlywheelHardwareConfig(new int[] {21}, new boolean[] {false}, 2.0, 60, "");
  public static final FlywheelGains RIGHT_CORAL_INTAKE_ROLLER_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  public static final FlywheelHardwareConfig LEFT_CORAL_INTAKE_ROLLERS_CONFG =
      new FlywheelHardwareConfig(new int[] {22}, new boolean[] {true}, 2.0, 60, "");
  public static final FlywheelGains LEFT_CORAL_INTAKE_ROLLER_GAINS =
      new FlywheelGains(0.2, 0.0, 0.0, 0.0, 0.065, 0.0, 1.0, 1.0);

  public static final FlywheelGains END_EFFECTOR_GAINS =
      new FlywheelGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  public static final FlywheelHardwareConfig END_EFFECTOR_CONFIG =
      new FlywheelHardwareConfig(new int[] {34}, new boolean[] {true}, 2.0, 40, "");

  public static final FlywheelGains CLIMBER_ROLLER_GAINS =
      new FlywheelGains(0, 0, 0, 0, 0, 0, 0, 0);
  public static final FlywheelHardwareConfig CLIMBER_ROLLER_CONFIG =
      new FlywheelHardwareConfig(new int[] {41}, new boolean[] {true}, 2.0, 20, "");
}

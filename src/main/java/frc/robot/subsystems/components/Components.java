package frc.robot.subsystems.components;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Components extends SubsystemBase {
  private final DoubleSupplier elevator;
  // private final DoubleSupplier pivot;
  // private final DoubleSupplier intakeRight;
  // private final DoubleSupplier intakeLeft;

  // private final LoggedTunableNumber elevator = new
  // LoggedTunableNumber("ElevatorSimPosition", 0);
  private final LoggedTunableNumber pivot = new LoggedTunableNumber("PivotSimPosition", 0);
  private final LoggedTunableNumber intakeRight =
      new LoggedTunableNumber("RightIntakeSimPosition", 0);
  private final LoggedTunableNumber intakeLeft =
      new LoggedTunableNumber("LeftIntakeSimPosition", 0);

  public Components(
      DoubleSupplier elevator,
      DoubleSupplier pivot,
      DoubleSupplier intakeRight,
      DoubleSupplier intakeLeft) {
    this.elevator = elevator;
    // this.pivot = pivot;
    // this.intakeRight = intakeRight;
    // this.intakeLeft = intakeLeft;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose3d[] poses = new Pose3d[10];

    poses[0] = new Pose3d(); // Base parts

    // Elevator
    poses[1] = new Pose3d(new Translation3d(0, 0, elevator.getAsDouble() / 2), new Rotation3d());
    poses[2] = new Pose3d(new Translation3d(0, 0, elevator.getAsDouble()), new Rotation3d());

    // Pivot
    poses[3] =
        new Pose3d(
            new Translation3d(0.40005 - 0.3048, 0, elevator.getAsDouble() + 0.31116367),
            new Rotation3d(0, Units.degreesToRadians(pivot.getAsDouble()), 0));

    // Intake Right
    poses[4] =
        new Pose3d(
            new Translation3d(0, -0.4699 + 0.3048, 0.3184652),
            new Rotation3d(Units.degreesToRadians(intakeRight.getAsDouble()), 0, 0));
    poses[5] =
        new Pose3d(
            new Translation3d(0, -0.55245 + 0.3048, 0.2752725),
            new Rotation3d(1.2 * Units.degreesToRadians(intakeRight.getAsDouble()), 0, 0));
    poses[6] =
        new Pose3d(
            new Translation3d(
                0,
                -0.47001475
                    + 0.3048
                    - 0.27256233 * Math.sin(Units.degreesToRadians(intakeRight.getAsDouble())),
                0.3184652
                    + 0.27256233 * Math.cos(Units.degreesToRadians(intakeRight.getAsDouble()))),
            new Rotation3d(0, 0, 0));

    // Intake Left
    poses[7] =
        new Pose3d(
            new Translation3d(0, -0.1397 + 0.3048, 0.3184652),
            new Rotation3d(Units.degreesToRadians(intakeLeft.getAsDouble()), 0, 0));
    poses[8] =
        new Pose3d(
            new Translation3d(0, -0.05715 + 0.3048, 0.2752725),
            new Rotation3d(Units.degreesToRadians(intakeLeft.getAsDouble()), 0, 0));
    poses[9] = new Pose3d(new Translation3d(0, -0.1105782 + 0.3048, 0.59102753), new Rotation3d());

    // Update the robot's pose
    Logger.recordOutput("Components", poses);
  }
}

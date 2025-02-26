package frc.robot.subsystems.components;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Components extends SubsystemBase {
  private final DoubleSupplier elevator;
  private final DoubleSupplier pivot;
  private final DoubleSupplier intakeRight;
  private final DoubleSupplier intakeLeft;

  public Components(
      DoubleSupplier elevator,
      DoubleSupplier pivot,
      DoubleSupplier intakeRight,
      DoubleSupplier intakeLeft) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.intakeRight = intakeRight;
    this.intakeLeft = intakeLeft;
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
            new Rotation3d(0, pivot.getAsDouble(), 0));

    // Intake Right
    poses[4] =
        new Pose3d(
            new Translation3d(0, -0.4699 + 0.3048, 0.3184652),
            new Rotation3d(intakeRight.getAsDouble() * 2.0 * Math.PI, 0, 0));
    poses[5] = new Pose3d();
    poses[6] = new Pose3d();

    // Intake Left
    poses[7] =
        new Pose3d(
            new Translation3d(0, -0.1397 + 0.3048, 0.3184652),
            new Rotation3d(intakeLeft.getAsDouble() * 2.0 * Math.PI, 0, 0));
    poses[8] = new Pose3d();
    poses[9] = new Pose3d();

    // Update the robot's pose
    Logger.recordOutput("Components", poses);
  }
}

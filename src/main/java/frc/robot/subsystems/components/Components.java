package frc.robot.subsystems.components;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.position_joint.PositionJoint;
import org.littletonrobotics.junction.Logger;

public class Components extends SubsystemBase {
  private final PositionJoint elevator;
  private final PositionJoint pivot;
  private final PositionJoint intakeRight;
  private final PositionJoint intakeLeft;

  public Components(
      PositionJoint elevator,
      PositionJoint pivot,
      PositionJoint intakeRight,
      PositionJoint intakeLeft) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.intakeRight = intakeRight;
    this.intakeLeft = intakeLeft;
  }

  public Components(PositionJoint intakeRight, PositionJoint intakeLeft) {
    this.elevator = null;
    this.pivot = null;
    this.intakeRight = intakeRight;
    this.intakeLeft = intakeLeft;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Pose3d[] poses = new Pose3d[10];

    poses[0] = new Pose3d(); // Base parts
    // Elevator
    // poses[0] = new Pose3d(new Translation3d(0, 0, elevator.getPosition() / 2),
    // new Rotation3d(0, 0, 0));
    // poses[1] = new Pose3d(new Translation3d(0, 0, elevator.getPosition()), new
    // Rotation3d(0, 0, 0));

    // Pivot
    // poses[2] = new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0,
    // pivot.getPosition()));

    /*
     * TODO: remove below for loop and replace with above code after system
     * implementation
     */
    for (int i = 1; i < 4; i++) {
      poses[i] = new Pose3d();
    }

    // Intake Right
    poses[4] =
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(intakeRight.getPosition(), 0, 0));
    poses[5] = new Pose3d();
    poses[6] = new Pose3d();

    // Intake Left
    poses[7] =
        new Pose3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, intakeLeft.getPosition()));
    poses[8] = new Pose3d();
    poses[9] = new Pose3d();

    // Update the robot's pose
    Logger.recordOutput("Components", poses);
  }
}

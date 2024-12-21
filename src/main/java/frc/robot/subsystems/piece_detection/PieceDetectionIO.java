package frc.robot.subsystems.piece_detection;

import org.littletonrobotics.junction.AutoLog;

public interface PieceDetectionIO {
  @AutoLog
  public static class PieceDetectionIOInputs {
    public boolean connected = false;

    public double yaw = 0.0;
    public double pitch = 0.0;
    public double area = 0.0;

    public boolean seesTarget = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PieceDetectionIOInputs inputs) {}

  public default String getName() {
    return "Piece Detection";
  }
}
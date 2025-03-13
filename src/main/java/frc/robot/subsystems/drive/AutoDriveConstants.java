package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.commands.CoralPreset;
import frc.robot.commands.ElevatorCommands;

public class AutoDriveConstants {
  public static Pose2d[][] reefPoses = {
    {
      new Pose2d(3.347, 4.181, Rotation2d.fromDegrees(0)), // A
      new Pose2d(3.730, 3.104, Rotation2d.fromDegrees(55.4)), // C
      new Pose2d(4.948, 2.877, Rotation2d.fromDegrees(119.1)), // E
      new Pose2d(5.685, 3.869, Rotation2d.fromDegrees(-178.6)), // G
      new Pose2d(5.232, 4.932, Rotation2d.fromDegrees(-117.8)), // I
      new Pose2d(4.041, 5.130, Rotation2d.fromDegrees(-61.3)), // K
    },
    {
      new Pose2d(3.304, 3.869, Rotation2d.fromDegrees(0)), // B
      new Pose2d(4.056, 2.948, Rotation2d.fromDegrees(54.7)), // D
      new Pose2d(5.232, 3.061, Rotation2d.fromDegrees(122.3)), // F
      new Pose2d(5.671, 4.181, Rotation2d.fromDegrees(-178.6)), // H
      new Pose2d(4.92, 5.102, Rotation2d.fromDegrees(-117.8)), // J
      new Pose2d(3.772, 4.954, Rotation2d.fromDegrees(-61.9)), // L
    }
  };
  public static CoralPreset[] coralPresets = {
    ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_ONE_CORAL,
    ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_TWO_CORAL,
    ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL,
    ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL,
    ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL,
    ElevatorCommands.ELEVATOR_HEIGHT_PRESETS.L_FOUR_CORAL,
  };

  public static double DISTANCE_THRESH = 0.5;
}

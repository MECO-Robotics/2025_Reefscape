package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Camera names, must match names configured on coprocessor
  // public static String camera0Name = "camera_0";
  // public static String camera1Name = "camera_1"
  public static String frontTagCamera = "front_tag_camera";
  public static String backTagCamera = "back_tag_camera";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToQuest =
      new Transform3d(
          0.0,
          Units.inchesToMeters(-10.5),
          Units.inchesToMeters(36),
          new Rotation3d(0.0, 0.0, Math.toRadians(180.0)));
  // public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2,
  // new Rotation3d(0.0, -0.4, Math.PI));
  public static Transform3d robotToLeftTagCamera =
      new Transform3d(0.00555625, 0.1651, 0.813, new Rotation3d(0, Math.toRadians(38), 0));
  public static Transform3d robotToRightTagCamera =
      new Transform3d(0.00555625, -0.1651, 0.813, new Rotation3d(0, Math.toRadians(38), 0));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 10;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}

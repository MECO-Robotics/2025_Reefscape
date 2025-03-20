package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class PieceDetectionConstants {
    public record PieceDetectionConfig(Transform3d robotToCameraTransform) {
    }

    public static final PieceDetectionConfig LEFT_CONFIG = new PieceDetectionConfig(
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-0.5), Units.inchesToMeters(4), Units.inchesToMeters(60)),
                    new Rotation3d(0, Units.degreesToRadians(120), Units.degreesToRadians(90))));

    public static final PieceDetectionConfig RIGHT_CONFIG = new PieceDetectionConfig(
            new Transform3d(
                    new Translation3d(
                            Units.inchesToMeters(-0.5), Units.inchesToMeters(-4), Units.inchesToMeters(60)),
                    new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(270))));
}

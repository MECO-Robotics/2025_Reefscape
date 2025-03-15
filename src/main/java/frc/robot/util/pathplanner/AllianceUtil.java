package frc.robot.util.pathplanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceUtil {
  public static Rotation2d flipRotation2dAlliance(Rotation2d rotation) {
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return rotation;
      } else {
        // return Rotation2d.fromDegrees(180).minus(rotation);
        return rotation.plus(Rotation2d.fromDegrees(180));
      }
    } else {
      return rotation;
    }
  }

  public static Rotation2d getRotationFromReefTagID(int tagID) {
    switch (tagID) {
      case 6:
        return Rotation2d.fromDegrees(-60);
      case 7:
        return Rotation2d.fromDegrees(0);
      case 8:
        return Rotation2d.fromDegrees(60);
      case 9:
        return Rotation2d.fromDegrees(120);
      case 10:
        return Rotation2d.fromDegrees(180);
      case 11:
        return Rotation2d.fromDegrees(-120);
      case 17:
        return Rotation2d.fromDegrees(60);
      case 18:
        return Rotation2d.fromDegrees(0);
      case 19:
        return Rotation2d.fromDegrees(-60);
      case 20:
        return Rotation2d.fromDegrees(-120);
      case 21:
        return Rotation2d.fromDegrees(180);
      case 22:
        return Rotation2d.fromDegrees(120);
      default:
        return Rotation2d.fromDegrees(0);
    }
  }

  public static int getTagIDFromReefAlliance(String reef) {
    boolean isBlue;

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        isBlue = true;
      } else {
        isBlue = false;
      }
    } else {
      isBlue = true;
    }

    switch (reef) {
      case "AB":
        return isBlue ? 18 : 7;
      case "CD":
        return isBlue ? 17 : 8;
      case "EF":
        return isBlue ? 22 : 9;
      case "GH":
        return isBlue ? 21 : 10;
      case "IJ":
        return isBlue ? 20 : 11;
      case "KL":
        return isBlue ? 19 : 6;
      default:
        return 0;
    }
  }
}

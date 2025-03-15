package frc.robot.util.controller;

public class ControllerUtil {
  public static int getIndex(double x, double y) {
    if (x * x + y * y < 0.3) {
      return 0;
    }
    return (int) Math.round(((Math.toDegrees(Math.atan2(y, x)) + 30) / 60)) % 6 + 2;
  }
}

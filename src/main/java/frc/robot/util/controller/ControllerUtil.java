package frc.robot.util.controller;

public class ControllerUtil {
  public static int getIndex(double x, double y) {

    return (int) Math.round(((Math.toDegrees(Math.atan2(y, x)) + 30) / 60)) % 6 + 2;
  }
}

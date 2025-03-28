package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

// Creates the logged tunable numbers for the elevator and wrist
// Super cool as we are able to call a command and this allows us to control both
// the wrist and the elevator positions

public class CoralPreset {
  private final LoggedTunableNumber elevatorPos;
  private final LoggedTunableNumber wristPos;

  public CoralPreset(String name, double elevatorPosMeters, Rotation2d wristPos) {
    this.elevatorPos =
        new LoggedTunableNumber("CoralPresets/" + name + "/elevatorPos", elevatorPosMeters);
    this.wristPos =
        new LoggedTunableNumber("CoralPresets/" + name + "/wristPos", wristPos.getDegrees());
  }

  public double getElevatorPos() {
    return elevatorPos.get();
  }

  public Rotation2d getWristPos() {
    return Rotation2d.fromDegrees(wristPos.get());
  }
}

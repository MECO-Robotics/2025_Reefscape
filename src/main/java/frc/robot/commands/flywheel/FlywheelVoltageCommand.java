package frc.robot.commands.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flywheel.Flywheel;
import java.util.function.DoubleSupplier;

public class FlywheelVoltageCommand extends Command {
  private Flywheel flywheel;
  private DoubleSupplier voltage;

  public FlywheelVoltageCommand(Flywheel flywheel, DoubleSupplier voltage) {
    this.flywheel = flywheel;
    this.voltage = voltage;
    addRequirements(flywheel);
  }

  public FlywheelVoltageCommand(Flywheel rollerMotor, int i) {
    // TODO Auto-generated constructor stub
  }

  @Override
  public void execute() {
    flywheel.setVoltage(voltage.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

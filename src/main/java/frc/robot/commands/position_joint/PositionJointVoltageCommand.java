package frc.robot.commands.position_joint;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.position_joint.PositionJoint;
import java.util.function.DoubleSupplier;

public class PositionJointVoltageCommand extends Command {
  private final PositionJoint positionJoint;

  private final DoubleSupplier voltageSupplier;

  public PositionJointVoltageCommand(PositionJoint positionJoint, DoubleSupplier voltageSupplier) {
    this.positionJoint = positionJoint;
    this.voltageSupplier = voltageSupplier;
  }

  @Override
  public void execute() {
    positionJoint.setVoltage(voltageSupplier.getAsDouble());
  }
}

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** A collection of commands for controlling the intake. */
public class IntakeCommands {

  /** Intake rotation preset positions. */
  public static final class ROTATION_POSITIONS {
    public static final LoggedTunableNumber STOW =
        new LoggedTunableNumber("CoralIntakeRotation", 0);
    public static final LoggedTunableNumber DEPLOY =
        new LoggedTunableNumber("CoralIntakePosition", Units.degreesToRotations(101));
  }

  /** Intake roller preset voltages. */
  public final class ROLLER_VOLTS {
    public static final LoggedTunableNumber INTAKE =
        new LoggedTunableNumber("CoralIntakeSpeed", -5);
    public static final LoggedTunableNumber STOW =
        new LoggedTunableNumber("CoralOuttakeSpeed", -1.5);
    public static final LoggedTunableNumber EMERGENCY =
        new LoggedTunableNumber("Emergency Outtake", -12);
  }

  /**
   * Stows the intake by moving the rotation motor to the up position and set ting the roller motor
   * to stow speed.
   */
  public static Command stowIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(rotationMotor, ROTATION_POSITIONS.STOW),
        new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.STOW));
  }

  public static Command emergencyOuttake(Flywheel rollerMotor) {
    return Commands.parallel(new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.EMERGENCY));
  }

  /**
   * Deploys the intake by moving the rotation motor to the down position and setting the roller
   * motor to intake speed.
   */
  public static Command deployIntake(PositionJoint rotationMotor, Flywheel rollerMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(rotationMotor, ROTATION_POSITIONS.DEPLOY),
        new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.INTAKE));
  }
}

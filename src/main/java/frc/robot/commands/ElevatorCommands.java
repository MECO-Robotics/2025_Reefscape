package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** A collection of commands for controlling the intake. */
public class ElevatorCommands {

  /** Intake rotation preset positions. */
  public static final class ROTATION_POSITIONS {
    public static final LoggedTunableNumber ELEVATOR_STOW =
        new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber HANDOFF =
        new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber L_ONE = new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber L_TWO = new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber L_THREE =
        new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber L_FOUR = new LoggedTunableNumber("ElevatorPosition", 0);
  }

  /** Intake roller preset voltages. */

  /**
   * Stows the intake by moving the rotation motor to the up position and setting the roller motor
   * to stow speed.
   */
  /*
   * public static Command stowEleator(PositionJoint rotationMotor) {
   * return Commands.parallel(
   * new PositionJointPositionCommand(rotationMotor, ROTATION_POSITIONS.STOW),
   * new FlywheelVoltageCommand(rollerMotor, ROLLER_VOLTS.STOW));
   * }
   */
  public static Command handOff(PositionJoint elevatorMotor) {
    return new PositionJointPositionCommand(elevatorMotor, ROTATION_POSITIONS.HANDOFF);
  }

  /**
   * Deploys the intake by moving the rotation motor to the down position and setting the roller
   * motor to intake speed.
   */
}

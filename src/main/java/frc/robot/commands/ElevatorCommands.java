package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** A collection of commands for controlling the intake. */
public class ElevatorCommands {

  /** Intake rotation preset positions. */
  public static final class ELEVATOR_HEIGHT_PRESETS {

    public static final LoggedTunableNumber HANDOFF =
        new LoggedTunableNumber("ElevatorPosition", 0.0);

    public static final LoggedTunableNumber L_ONE = new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber L_TWO = new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber L_THREE =
        new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber L_FOUR = new LoggedTunableNumber("ElevatorPosition", 0);
    public static final LoggedTunableNumber MAX_POS =
        new LoggedTunableNumber("ElevatorPosition", 1.27);
  }

  public static final class WRIST_PRESETS {
    public static final LoggedTunableNumber WRIST_STOW =
        new LoggedTunableNumber("WristPosition", 0);

    public static final LoggedTunableNumber WRIST_HANDOFF =
        new LoggedTunableNumber("WristPosition", 0);

    public static final LoggedTunableNumber WRIST_L_ONE =
        new LoggedTunableNumber("WristPosition", 0);
    public static final LoggedTunableNumber WRIST_L_TWO =
        new LoggedTunableNumber("WristPosition", 0);
    public static final LoggedTunableNumber WRIST_L_THREE =
        new LoggedTunableNumber("WristPosition", 0);
    public static final LoggedTunableNumber WRIST_L_FOUR =
        new LoggedTunableNumber("WristPosition", 0);
  }

  public static final class END_EFFECTOR_PRESETS {
    public static LoggedTunableNumber END_EFFECTOR_OUTTAKE =
        new LoggedTunableNumber("EndEffectorVelocity", 0);
    public static LoggedTunableNumber END_EFFECTOR_INTAKE =
        new LoggedTunableNumber("EndEffectorVelocity", 0);
  }

  public static Command stowEleator(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return Commands.parallel(
        // new PositionJointPositionCommand(elevatorMotor,
        // ELEVATOR_HEIGHT_PRESETS.ELEVATOR_STOW),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_STOW));
  }

  public static Command L_ONE_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_ONE),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_ONE));
  }

  public static Command L_TWO_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_TWO),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_TWO));
  }

  public static Command L_THREE_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_THREE),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_THREE));
  }

  public static Command L_FOUR_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return Commands.parallel(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_FOUR),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_FOUR));
  }

  public static Command MAX(PositionJoint elevatormotor) {
    return new PositionJointPositionCommand(elevatormotor, ELEVATOR_HEIGHT_PRESETS.MAX_POS);
  }

  public static Command handOff(PositionJoint elevatorMotor) {
    return new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.HANDOFF);
  }
}

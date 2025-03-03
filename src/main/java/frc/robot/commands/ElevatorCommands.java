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

    public static final LoggedTunableNumber HANDOFF = new LoggedTunableNumber("Presets/ElevatorPosition/Handoff", 0.0);

    public static final LoggedTunableNumber L_ONE = new LoggedTunableNumber("Presets/ElevatorPosition/L1", 0.4);
    public static final LoggedTunableNumber L_TWO = new LoggedTunableNumber("Presets/ElevatorPosition/L2", 0.6);
    public static final LoggedTunableNumber L_THREE = new LoggedTunableNumber("Presets/ElevatorPosition/L3", 0.8);
    public static final LoggedTunableNumber L_FOUR = new LoggedTunableNumber("Presets/ElevatorPosition/L4", 01);
    public static final LoggedTunableNumber MAX_POS = new LoggedTunableNumber("Presets/ElevatorPosition/Max", 1.27);
  }

  public static final class WRIST_PRESETS {
    public static final LoggedTunableNumber WRIST_STOW = new LoggedTunableNumber("Presets/WristPosition/Stow", 0);

    public static final LoggedTunableNumber WRIST_HANDOFF = new LoggedTunableNumber("Presets/WristPosition/Handoff", 0);

    public static final LoggedTunableNumber WRIST_L_ONE = new LoggedTunableNumber("Presets/WristPosition/L1", 0);
    public static final LoggedTunableNumber WRIST_L_TWO = new LoggedTunableNumber("Presets/WristPosition/L2", 0);
    public static final LoggedTunableNumber WRIST_L_THREE = new LoggedTunableNumber("Presets/WristPosition/L3", 0);
    public static final LoggedTunableNumber WRIST_L_FOUR = new LoggedTunableNumber("Presets/WristPosition/L4", 0);
  }

  public static final class END_EFFECTOR_PRESETS {
    public static LoggedTunableNumber END_EFFECTOR_OUTTAKE = new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
    public static LoggedTunableNumber END_EFFECTOR_INTAKE = new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
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

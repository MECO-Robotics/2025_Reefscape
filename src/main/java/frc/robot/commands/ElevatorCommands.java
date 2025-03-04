package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** A collection of commands for controlling the intake. */
public class ElevatorCommands {

  /** Intake rotation preset positions. */
  public static final class ELEVATOR_HEIGHT_PRESETS {

    public static final LoggedTunableNumber HANDOFF =
        new LoggedTunableNumber("Presets/ElevatorPosition/Handoff", 0.0);

    public static final LoggedTunableNumber MID =
        new LoggedTunableNumber("Presets/ElevatorPosition/Middle", 0.635);

    public static final LoggedTunableNumber L_ONE =
        new LoggedTunableNumber("Presets/ElevatorPosition/L1", 0.276);
    public static final LoggedTunableNumber L_TWO =
        new LoggedTunableNumber("Presets/ElevatorPosition/L2", 0.586);
    public static final LoggedTunableNumber L_THREE =
        new LoggedTunableNumber("Presets/ElevatorPosition/L3", 0.268);
    public static final LoggedTunableNumber L_FOUR =
        new LoggedTunableNumber("Presets/ElevatorPosition/L4", 0.939);
    public static final LoggedTunableNumber MAX_POS =
        new LoggedTunableNumber("Presets/ElevatorPosition/Max", 1.27);
  }

  public static final class WRIST_PRESETS {
    public static final LoggedTunableNumber WRIST_STOW =
        new LoggedTunableNumber("Presets/WristPosition/Stow", -2.94);

    public static final LoggedTunableNumber WRIST_HANDOFF =
        new LoggedTunableNumber("Presets/WristPosition/Handoff", -2.94);

    // 108.05 used for placement rotations/angle (will add later)
    public static final LoggedTunableNumber WRIST_L_ONE =
        new LoggedTunableNumber("Presets/WristPosition/L1", 26.67);
    public static final LoggedTunableNumber WRIST_L_TWO =
        new LoggedTunableNumber("Presets/WristPosition/L2", 9.68);
    public static final LoggedTunableNumber WRIST_L_THREE =
        new LoggedTunableNumber("Presets/WristPosition/L3", 161.65);
    public static final LoggedTunableNumber WRIST_L_FOUR =
        new LoggedTunableNumber("Presets/WristPosition/L4", 161.71);
    public static final LoggedTunableNumber WRIST_MAX =
        new LoggedTunableNumber("Presets/WristPosition/L4", 207.91);
  }

  public static final class END_EFFECTOR_PRESETS {
    public static LoggedTunableNumber END_EFFECTOR_OUTTAKE =
        new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
    public static LoggedTunableNumber END_EFFECTOR_INTAKE =
        new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
  }

  public static Command stowEleator(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return Commands.parallel(
        // new PositionJointPositionCommand(elevatorMotor,
        // ELEVATOR_HEIGHT_PRESETS.ELEVATOR_STOW),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_STOW));
  }

  public static Command L_ONE_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.MID),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_ONE),
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_ONE));
  }

  public static Command L_TWO_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.MID),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_TWO),
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_TWO));
  }

  public static Command L_THREE_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.MID),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_THREE),
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_THREE));
  }

  public static Command L_FOUR_POSITION(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.MID),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_FOUR),
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.L_FOUR));
  }

  public static Command MIN(PositionJoint elevatormotor) {
    return new PositionJointPositionCommand(elevatormotor, ELEVATOR_HEIGHT_PRESETS.HANDOFF);
  }

  public static Command MAX(PositionJoint elevatormotor) {
    return new PositionJointPositionCommand(elevatormotor, ELEVATOR_HEIGHT_PRESETS.MAX_POS);
  }

  public static Command HANDOFF(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.MID),
        new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_HANDOFF),
        new PositionJointPositionCommand(elevatorMotor, ELEVATOR_HEIGHT_PRESETS.HANDOFF));
  }

  public static Command WRIST_MAX(PositionJoint wristMotor) {
    return new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_FOUR);
  }

  public static Command WRIST_LOW_TEST(PositionJoint wristMotor) {
    return new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_ONE);
  }
}

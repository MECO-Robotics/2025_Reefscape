package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

/** A collection of commands for controlling the intake. */
public class ElevatorCommands {

  /** Intake rotation preset positions. */
  public static final class ELEVATOR_HEIGHT_PRESETS {

    public static final CoralPreset HANDOFF =
        new CoralPreset("HandOff", 0, Rotation2d.fromRotations(-0.25));

    public static final CoralPreset STOW =
        new CoralPreset("Stow", 0.0, Rotation2d.fromRotations(0.25));

    public static final CoralPreset MID =
        new CoralPreset("Mid", 0.635, Rotation2d.fromRotations(0));

    public static final CoralPreset L_ONE_CORAL =
        new CoralPreset("L1", 0.35, Rotation2d.fromRotations(-0.21));

    public static final CoralPreset L_TWO_CORAL =
        new CoralPreset("L2", 0.55, Rotation2d.fromRotations(-0.21));

    public static final CoralPreset L_THREE_CORAL =
        new CoralPreset("L3", 0.5, Rotation2d.fromRotations(0.15));

    public static final CoralPreset L_FOUR_CORAL =
        new CoralPreset("L4", 1.01, Rotation2d.fromRotations(0.16));

    public static final CoralPreset L_THREE_SCORE =
        new CoralPreset("L3_Score", 0.5, Rotation2d.fromRotations(0.15));

    public static final CoralPreset L_FOUR_SCORE =
        new CoralPreset("L4_Score", 1.01, Rotation2d.fromRotations(0.16));

    public static final CoralPreset WAIT_FOR_CORAL =
        new CoralPreset("Wait_For_Coral", 0.5, Rotation2d.fromRotations(-0.25));
  }

  public static final LoggedTunableNumber tinyBitDown =
      new LoggedTunableNumber("Tiny bit down", 0.1);

  public static final Command setPreset(
      PositionJoint elevatorMotor, PositionJoint wristMotor, CoralPreset preset) {
    // move elevator to L3, move wrist, then down or up depending on where to score
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            elevatorMotor, () -> ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL.getElevatorPos()),
        new PositionJointPositionCommand(wristMotor, () -> preset.getWristPos().getRotations()),
        new PositionJointPositionCommand(elevatorMotor, () -> preset.getElevatorPos()));
  }

  public static final Command scorePreset(PositionJoint elevatorMotor, PositionJoint wristMotor) {
    // move wrist first, then possibly move to reset for next coral (add soon)
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            wristMotor, () -> wristMotor.getDesiredPosition() - tinyBitDown.getAsDouble()));
  }

  public static final Command scoreReattempt(
      PositionJoint elevatorMotor, PositionJoint wristMotor) {
    // move wrist first, then possibly move to reset for next coral (add soon)
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            wristMotor, () -> wristMotor.getDesiredPosition() + tinyBitDown.getAsDouble()));
  }

  public static final class END_EFFECTOR_PRESETS {
    public static LoggedTunableNumber END_EFFECTOR_OUTTAKE =
        new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
    public static LoggedTunableNumber END_EFFECTOR_INTAKE =
        new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
  }
}

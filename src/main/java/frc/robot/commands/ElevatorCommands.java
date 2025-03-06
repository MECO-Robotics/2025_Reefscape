package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final CoralPreset HANDOFF = new CoralPreset("HandOff", 0, null);

    public static final CoralPreset STOW = new CoralPreset("Stow", 0.0, new Rotation2d(0));

    public static final CoralPreset MID = new CoralPreset("Mid", 0.635, new Rotation2d(0));

    public static final CoralPreset L_ONE_CORAL = new CoralPreset("L1", 0.35, new Rotation2d(0));

    public static final CoralPreset L_TWO_CORAL = new CoralPreset("L2", 0.55, new Rotation2d(0));

    public static final CoralPreset L_THREE_CORAL = new CoralPreset("L3", 0.5, new Rotation2d(0));

    public static final CoralPreset L_FOUR_CORAL = new CoralPreset("L4", 1.01, new Rotation2d(0));
  }

  public static final class WRIST_PRESETS {
    public static final LoggedTunableNumber WRIST_STOW = new LoggedTunableNumber("Presets/WristPosition/Stow", -2.94);

    public static final LoggedTunableNumber WRIST_HANDOFF = new LoggedTunableNumber("Presets/WristPosition/Handoff",
        -2.94);

    public static final LoggedTunableNumber WRIST_L_ONE = new LoggedTunableNumber("Presets/WristPosition/L1", -0.21);
    public static final LoggedTunableNumber WRIST_L_TWO = new LoggedTunableNumber("Presets/WristPosition/L2", -0.21);
    public static final LoggedTunableNumber WRIST_L_THREE = new LoggedTunableNumber("Presets/WristPosition/L3", 0.15);
    public static final LoggedTunableNumber WRIST_L_FOUR = new LoggedTunableNumber("Presets/WristPosition/L4", 0.16);
    public static final LoggedTunableNumber WRIST_MAX = new LoggedTunableNumber("Presets/WristPosition/L4", 207.91);
  }

  public static final Command CoralPreset(PositionJoint elevatorMotor, PositionJoint wristMotor,
      CoralPreset preset) {
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(elevatorMotor, () -> preset.getElevatorPos()),
        new PositionJointPositionCommand(wristMotor, () -> preset.getWristPos().getRotations()));
  }

  public static final class END_EFFECTOR_PRESETS {
    public static LoggedTunableNumber END_EFFECTOR_OUTTAKE = new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
    public static LoggedTunableNumber END_EFFECTOR_INTAKE = new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
  }

  public static Command WRIST_MAX(PositionJoint wristMotor) {
    return new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_FOUR);
  }

  public static Command WRIST_LOW_TEST(PositionJoint wristMotor) {
    return new PositionJointPositionCommand(wristMotor, WRIST_PRESETS.WRIST_L_ONE);
  }
}

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.function.Supplier;

/** A collection of commands for controlling the intake. */
public class ElevatorCommands {

  /** Intake rotation preset positions. */
  public static final class ELEVATOR_HEIGHT_PRESETS {

    public static final CoralPreset HANDOFF =
        new CoralPreset("HandOff", -0.083, Rotation2d.fromRotations(-0.25));

    public static final CoralPreset STOW =
        new CoralPreset("Stow", 0.0, Rotation2d.fromRotations(0.25));

    public static final CoralPreset SAFE =
        new CoralPreset("SAFE", 0.25, Rotation2d.fromRotations(0));

    public static final CoralPreset L_ONE_CORAL =
        new CoralPreset("L1", 0.35, Rotation2d.fromRotations(-0.21));

    public static final CoralPreset L_TWO_CORAL =
        new CoralPreset("L2", 0.15, Rotation2d.fromDegrees(30));

    public static final CoralPreset L_THREE_CORAL =
        new CoralPreset("L3", 0.41, Rotation2d.fromRotations(0.15));

    public static final CoralPreset L_FOUR_CORAL =
        new CoralPreset("L4", 0.95, Rotation2d.fromRotations(0.16));

    public static final LoggedTunableNumber WRIST_HANDOFF =
        new LoggedTunableNumber("Presets/WristPosition/Handoff", 0);

    public static final LoggedTunableNumber WRIST_L_ONE =
        new LoggedTunableNumber("Presets/WristPosition/L1", 0);
    public static final LoggedTunableNumber WRIST_L_TWO =
        new LoggedTunableNumber("Presets/WristPosition/L2", 0);
    public static final LoggedTunableNumber WRIST_L_THREE =
        new LoggedTunableNumber("Presets/WristPosition/L3", 0);
    public static final LoggedTunableNumber WRIST_L_FOUR =
        new LoggedTunableNumber("Presets/WristPosition/L4", 0);

    public static final CoralPreset WAIT_FOR_CORAL =
        new CoralPreset("Wait_for_Coral", 0.2, Rotation2d.fromRotations(-0.25));
  }

  public static final class END_EFFECTOR_PRESETS {
    public static LoggedTunableNumber END_EFFECTOR_OUTTAKE =
        new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
    public static LoggedTunableNumber END_EFFECTOR_INTAKE =
        new LoggedTunableNumber("Presets/EndEffectorVelocity", 0);
  }

  public static final LoggedTunableNumber tinyBitDown =
      new LoggedTunableNumber("Tiny Bit Down", 0.1);

  public static final Command setPreset(
      PositionJoint elevatorMotor, PositionJoint wristMotor, CoralPreset preset) {
    // move elevator to L3, move wrist, then down or up depending on where to score
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            elevatorMotor, () -> ELEVATOR_HEIGHT_PRESETS.L_THREE_CORAL.getElevatorPos()),
        new PositionJointPositionCommand(wristMotor, () -> preset.getWristPos().getRotations()),
        new PositionJointPositionCommand(elevatorMotor, () -> preset.getElevatorPos()));
  }

  public static Command fromHandoff(
      PositionJoint elevatorMotor, PositionJoint wristMotor, CoralPreset preset) {
    return Commands.either(
        // if elevator is above 0, move the elevator
        PositionJoint.setPosition(elevatorMotor, () -> preset.getElevatorPos())
            .alongWith(
                // wait until elevator is above the safe point, then move the wrist
                Commands.waitUntil(
                        () ->
                            elevatorMotor.getPosition()
                                > ELEVATOR_HEIGHT_PRESETS.SAFE.getElevatorPos())
                    .andThen(
                        PositionJoint.setPosition(
                            wristMotor, () -> preset.getWristPos().getRotations())),
                new PrintCommand("From Handoff below")),
        // if elevator is below 0, then move elevator to the safe position
        Commands.parallel(
            PositionJoint.setPosition(
                    elevatorMotor, () -> ELEVATOR_HEIGHT_PRESETS.SAFE.getElevatorPos())
                // move wrist right after in safe position
                .andThen(
                    PositionJoint.setPosition(wristMotor, () -> preset.getWristPos().getRotations())
                        // wait for the wrist position to be 0 (horizontal position), then move
                        // the
                        // elevator again
                        .alongWith(
                            Commands.waitUntil(
                                () ->
                                    wristMotor.getPosition()
                                        > ELEVATOR_HEIGHT_PRESETS
                                            .SAFE
                                            .getWristPos()
                                            .getRotations()))
                        .andThen(
                            PositionJoint.setPosition(
                                elevatorMotor, () -> preset.getElevatorPos()))),
            new PrintCommand("From Handoff above")),
        // check and see if the elevator is above 0
        () -> preset.getElevatorPos() > ELEVATOR_HEIGHT_PRESETS.HANDOFF.getElevatorPos());
  }

  public static Command handOff(
      PositionJoint elevatorMotor, PositionJoint wristMotor, Flywheel endEffectorMotor) {
    // move the elevator to the handoff posititon
    return Commands.waitUntil(
            () -> elevatorMotor.getPosition() > ELEVATOR_HEIGHT_PRESETS.HANDOFF.getElevatorPos())
        // move the elevator slightly up as we bring the wrist down to pickup
        .andThen(
            PositionJoint.setPosition(
                wristMotor, () -> ELEVATOR_HEIGHT_PRESETS.HANDOFF.getWristPos().getRotations()))
        // when the wrist is done, this command ends, even if it doesn't reach it's
        // point
        .deadlineFor(
            PositionJoint.setPosition(
                elevatorMotor, () -> ELEVATOR_HEIGHT_PRESETS.SAFE.getElevatorPos()),
            new PrintCommand("To Handoff"))
        // elevator goes down to pickup the piece.
        .andThen(
            PositionJoint.setPosition(
                    elevatorMotor, () -> ELEVATOR_HEIGHT_PRESETS.HANDOFF.getElevatorPos())
                .alongWith(Flywheel.setVoltage(endEffectorMotor, () -> 6.0)))
        .andThen(new WaitCommand(0.5))
        .andThen(
            PositionJoint.setPosition(
                    elevatorMotor, () -> ELEVATOR_HEIGHT_PRESETS.WAIT_FOR_CORAL.getElevatorPos())
                .alongWith(Flywheel.setVoltage(endEffectorMotor, () -> 0.2)));
  }

  public static Command moveSafe(
      PositionJoint elevatorMotor, PositionJoint wristMotor, CoralPreset preset) {
    return Commands.either(
        // if the positions of both is in handoff, run fromHandoff
        fromHandoff(elevatorMotor, wristMotor, preset),
        // if not, move the elevator and the wrist to it's posititon at the same time.
        PositionJoint.setPosition(elevatorMotor, () -> preset.getElevatorPos())
            .alongWith(
                PositionJoint.setPosition(wristMotor, () -> preset.getWristPos().getRotations())),
        // check if the current position of the wrist and elevator is the handoff
        // position
        () ->
            elevatorMotor.getDesiredPosition() == ELEVATOR_HEIGHT_PRESETS.HANDOFF.getElevatorPos()
                && wristMotor.getDesiredPosition()
                    == ELEVATOR_HEIGHT_PRESETS.HANDOFF.getWristPos().getRotations());
  }

  public static Command moveSequential(
      PositionJoint elevatorMotor, PositionJoint wristMotor, Supplier<CoralPreset> preset) {
    return Commands.sequence(
        PositionJoint.setPosition(elevatorMotor, () -> preset.get().getElevatorPos()),
        PositionJoint.setPosition(wristMotor, () -> preset.get().getWristPos().getRotations()));
  }

  public static Command moveSafe(
      PositionJoint elevatorMotor, PositionJoint wristMotor, Supplier<CoralPreset> preset) {
    return Commands.either(
        // if the positions of both is in handoff, run fromHandoff
        fromHandoff(elevatorMotor, wristMotor, preset.get()),
        // if not, move the elevator and the wrist to it's posititon at the same time.
        PositionJoint.setPosition(elevatorMotor, () -> preset.get().getElevatorPos())
            .alongWith(
                PositionJoint.setPosition(
                    wristMotor, () -> preset.get().getWristPos().getRotations())),
        // check if the current position of the wrist and elevator is the handoff
        // position
        () ->
            elevatorMotor.getDesiredPosition() == ELEVATOR_HEIGHT_PRESETS.HANDOFF.getElevatorPos()
                && wristMotor.getDesiredPosition()
                    == ELEVATOR_HEIGHT_PRESETS.HANDOFF.getWristPos().getRotations());
  }

  public static final Command scorePreset(PositionJoint wristMotor) {
    // move wrist down to place and score (remember to back up!)
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            wristMotor, () -> wristMotor.getDesiredPosition() - tinyBitDown.getAsDouble()));
  }

  public static final Command scoreReattempt(PositionJoint wristMotor) {
    // move wrist up to try again
    return new SequentialCommandGroup(
        new PositionJointPositionCommand(
            wristMotor, () -> wristMotor.getDesiredPosition() + tinyBitDown.getAsDouble()));
  }
}

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class AutoAimShootCommand extends ParallelCommandGroup {
    public AutoAimShootCommand(EndEffectorSubsystem endeffectorSubsystem,
                               ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, BooleanSupplier stop) {
        addRequirements(endeffectorSubsystem, elevatorSubsystem, intakeSubsystem);
        addCommands(
                Commands.race(
                        new WaitUntilCommand(stop),
                        Commands.sequence(
                                new ReefAimCommand(stop),
                                Commands.sequence(
                                        Commands.deadline(
                                                Commands.sequence(
                                                        Commands.waitSeconds(0.1),// wait to refresh the elevator target
                                                        new WaitUntilCommand(() -> (
                                                                elevatorSubsystem.elevatorReady(0.005) &&
                                                                        endeffectorSubsystem.isShootReady())),
                                                        Commands.waitSeconds(0.1)
                                                ),
                                                new PreShootCommand(endeffectorSubsystem, intakeSubsystem, elevatorSubsystem)
                                        ),
                                        new ShootCommand(endeffectorSubsystem)
                                )
                        )
                )
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}

package frc.robot.commands.manualSequence;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PutCoralCommand extends ParallelCommandGroup {
    public PutCoralCommand(CommandXboxController driverController, EndEffectorArmSubsystem endeffectorArmSubsystem,
                           ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, IndicatorSubsystem indicatorSubsystem) {
        addRequirements(endeffectorArmSubsystem, elevatorSubsystem, intakeSubsystem);
        addCommands(
                Commands.deadline(
                        new PreShootCommand(indicatorSubsystem, endeffectorArmSubsystem, intakeSubsystem, elevatorSubsystem),
                        Commands.sequence(
                                new WaitUntilCommand(() -> (
                                        driverController.rightTrigger().getAsBoolean() &&
                                                endeffectorArmSubsystem.isShootReady())),
                                new ShootCommand(indicatorSubsystem, endeffectorArmSubsystem)
                        )
                )
        );
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}

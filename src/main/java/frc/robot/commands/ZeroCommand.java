package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem.WantedState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;


public class ZeroCommand extends SequentialCommandGroup {
    public ZeroCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem , ClimberSubsystem climberSubsystem) {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> { endEffectorSubsystem.setWantedState(WantedState.IDLE); System.out.println("Finished setting end effector state"); }),
                        Commands.sequence(
                                Commands.runOnce(() -> { intakeSubsystem.setWantedState(frc.robot.subsystems.intake.IntakeSubsystem.WantedState.GROUNDZERO); System.out.println("Finished setting intaker state"); }),
                                Commands.runOnce(() -> { elevatorSubsystem.setElevatorState(frc.robot.subsystems.elevator.ElevatorSubsystem.WantedState.ZERO); System.out.println("Finished setting elevator state"); })
                        )
                )
        );
    }
}

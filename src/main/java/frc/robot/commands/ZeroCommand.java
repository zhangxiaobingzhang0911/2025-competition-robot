package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem.WantedState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;

import static frc.robot.RobotConstants.IntakeConstants.FUNNEL_AVOID_ANGLE;

public class ZeroCommand extends SequentialCommandGroup {
    public ZeroCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem, ClimberSubsystem climberSubsystem) {
        addCommands(
                Commands.parallel(
                        Commands.runOnce(() -> endEffectorSubsystem.setWantedState(WantedState.IDLE)),
                        new ClimberInitiateCommand(climberSubsystem),
                        Commands.sequence(
                                Commands.runOnce(() -> intakeSubsystem.setWantedState(frc.robot.subsystems.intake.IntakeSubsystem.WantedState.FUNNEL_AVOID)),
                                new WaitUntilCommand(() -> intakeSubsystem.isNearAngle(FUNNEL_AVOID_ANGLE.get())),
                                Commands.runOnce(() -> elevatorSubsystem.setElevatorState(frc.robot.subsystems.elevator.ElevatorSubsystem.WantedState.ZERO)),
                                Commands.runOnce(() -> intakeSubsystem.setWantedState(frc.robot.subsystems.intake.IntakeSubsystem.WantedState.GROUNDZERO))
                        )
                )
        );
        addRequirements(elevatorSubsystem, intakeSubsystem, endEffectorSubsystem);
    }
}

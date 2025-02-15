package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class GroundIntakeCommandGroup extends ParallelCommandGroup {
    public GroundIntakeCommandGroup(IntakeSubsystem intakeSubsystem, EndEffectorSubsystem endEffectorSubsystem, ElevatorSubsystem elevatorSubsystem) {
        addCommands(
            Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.DEPLOY_INTAKE)),
            Commands.sequence(
                    new WaitUntilCommand(() -> intakeSubsystem.isNearAngle(RobotConstants.IntakeConstants.DEPLOY_ANGLE.get())),
                    new GroundIntakeCommand(intakeSubsystem, endEffectorSubsystem, elevatorSubsystem)
            )
        );
    }
}
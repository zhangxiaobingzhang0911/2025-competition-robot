package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class GroundOuttakeCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;

    public GroundOuttakeCommand(IntakeSubsystem intakeSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        addRequirements(intakeSubsystem, endEffectorArmSubsystem);
    }

    @Override
    public void execute() {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.OUTTAKE);
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.CORAL_OUTTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setWantedState(IntakeSubsystem.WantedState.HOME);
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelSelf;
    }
}

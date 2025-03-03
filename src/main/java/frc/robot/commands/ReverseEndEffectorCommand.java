package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

public class ReverseEndEffectorCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private final Timer timer = new Timer();

    public ReverseEndEffectorCommand(EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
    }

    @Override
    public void initialize() {
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.REVERSE);
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.1);
    }
}

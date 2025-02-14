package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCoastCommand extends Command {
    private final ClimberSubsystem climber;

    public ClimberCoastCommand(ClimberSubsystem climber) {
        this.climber = climber;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climber.setCoast();
        climber.resetPosition();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climber.setBrake();
    }
}

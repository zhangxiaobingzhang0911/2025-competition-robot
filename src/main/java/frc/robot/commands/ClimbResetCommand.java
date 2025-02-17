//This is the command that reset the motor position of climber
//and set the neutral mode to coast
//It is triggered by user button on roborio
//It should be used to release climber after endgame
//and to reset position according to mark on climber rope before game

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbResetCommand extends Command {
    private final ClimberSubsystem climber;

    public ClimbResetCommand(ClimberSubsystem climber) {
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

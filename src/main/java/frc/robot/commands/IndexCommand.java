package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.RobotConstants.EndEffectorConstants.*;

public class IndexCommand extends Command {
    private final EndEffectorSubsystem endEffectorSubsystem;
    private boolean enabledBefore = false;
    private double indexRPS = INDEX_RPS.get();
    private double holdRPS = HOLD_RPS.get();
    private double ShootRPS = SHOOT_RPS.get();
    private double SpitRPS = SPIT_RPS.get();

    public IndexCommand(
            EndEffectorSubsystem endEffectorSubsystem) {
        this.endEffectorSubsystem = endEffectorSubsystem;
        addRequirements(endEffectorSubsystem);
    }

    @Override
    public void initialize() {
        enabledBefore = false;
    }

    @Override
    public void execute() {
        if (!endEffectorSubsystem.getShootBeambreak()) {
            endEffectorSubsystem.setVoltage(8);
        }
        if (endEffectorSubsystem.getShootBeambreak()) {
            enabledBefore = true;
        }else {
            endEffectorSubsystem.setVoltage(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorSubsystem.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return !endEffectorSubsystem.getIntakeBeambreak() && endEffectorSubsystem.getShootBeambreak();
    }
}
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.indicator.IndicatorSubsystem;

public class AlgaeShootCommand extends Command {
    private final EndEffectorArmSubsystem endEffectorArmSubsystem;
    private final IndicatorSubsystem indicatorSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public AlgaeShootCommand(IndicatorSubsystem indicatorSubsystem, EndEffectorArmSubsystem endEffectorArmSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.endEffectorArmSubsystem = endEffectorArmSubsystem;
        this.indicatorSubsystem = indicatorSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem, endEffectorArmSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.NET_EXTENSION_METERS.get());
        if(elevatorSubsystem.getIo().isNearExtension(RobotConstants.ElevatorConstants.NET_EXTENSION_METERS.get())){
            endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_SHOOT);
        }
    }

    @Override
    public boolean isFinished() {
        return Robot.isSimulation() || endEffectorArmSubsystem.isShootFinished();
    }

    @Override
    public void end(boolean interrupted) {
        endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOME);
    }
}
